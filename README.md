# QB3rt host Nav2

Host-side Nav2 configs and launch files for QB3rt.
Nav2 runs here; the robot runs the rest onboard.

| where  | what                                                                 | TF it owns                |
|--------|----------------------------------------------------------------------|---------------------------|
| robot  | base driver + IMU + ORB-SLAM3 VIO + EKF + RPLIDAR + slam_toolbox     | `odom->base_footprint` (EKF), `map->odom` (slam_toolbox), URDF statics |
| host   | Nav2 (planner/controller/behaviors/BT, velocity smoother, collision monitor) + RViz | none                      |

The host's final `/cmd_vel` travels over WiFi to the wave_rover bridge on the
robot. The bridge's `cmd_timeout: 0.5` watchdog stops the wheels if the link
drops mid-drive.

## Layout

```
qb3rt_host/
├── qb3rt_env.sh              # ROS domain, RMW, CycloneDDS URI
├── nav2_host.launch.py       # Nav2 + RViz bringup
├── nav2_host.yaml            # Nav2 parameters
├── behavior_trees/           # no-spin BT XMLs (skid-steer cannot pivot)
├── LICENSE
└── README.md
```

## One-time host setup

1. Install Nav2:

   ```bash
   sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup
   ```

2. Clone or copy this repo to `~/qb3rt_host` (or keep it where it is and
   adjust the paths below). If the robot is mounted at `~/mnt/rb3`:

   ```bash
   bash ~/mnt/rb3/root/QB3rt/host/install_on_host.sh
   ```

   That script copies configs + the no-spin behavior trees to `~/qb3rt_host`.
   Re-run it after editing the canonical copies on the robot, or edit this
   checkout directly.

3. Create `~/cyclonedds.xml` on the host — see
   [CycloneDDS setup](#cyclonedds-setup) below. The robot side is
   `/opt/cyclonedds.xml` via `rover_env.sh`.

4. Edit `qb3rt_env.sh` if needed so `CYCLONEDDS_URI` points at your local
   CycloneDDS XML.

## CycloneDDS setup

Discovery runs over **unicast only** (multicast is unreliable on the rover's
WiFi AP), so the host and robot must each list the other as an explicit peer.
A robot that isn't in `<Peers>` is never probed and its topics silently never
appear — no error anywhere. `ros2 topic list` showing only
`/parameter_events` + `/rosout` while the robot pings fine is the signature
of a missing/wrong peer entry.

Create `~/cyclonedds.xml` (the path `CYCLONEDDS_URI` in `qb3rt_env.sh`
points at):

```xml
<?xml version="1.0" encoding="UTF-8"?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain Id="42">
    <General>
      <Interfaces>
        <!-- host link on the rover AP (192.168.0.0/24) -->
        <NetworkInterface name="wlp4s0" priority="default" multicast="default"/>
      </Interfaces>
      <AllowMulticast>false</AllowMulticast>
    </General>
    <Discovery>
      <ParticipantIndex>auto</ParticipantIndex>
      <!-- With multicast off, unicast discovery only probes participant
           indexes 0..MaxAutoParticipantIndex (default 9) per peer address.
           Nav2 alone exceeds that; without this, host-local nodes (and CLI
           tools) randomly fail to discover each other. -->
      <MaxAutoParticipantIndex>120</MaxAutoParticipantIndex>
      <Peers>
        <!-- localhost: required for host-local node-to-node discovery
             (multicast is off) - without it the Nav2 servers and RViz running
             here never discover each other, only the robot. -->
        <Peer address="localhost"/>
        <!-- the robot -->
        <Peer address="192.168.0.100"/>
      </Peers>
    </Discovery>
    <Internal>
      <!-- Headroom for large samples from the robot (/map, costmaps). The
           kernel caps this at net.core.rmem_max (Ubuntu default ~208 kB):
           raise it with
             sudo sysctl -w net.core.rmem_max=33554432
           and persist in /etc/sysctl.d/ to actually get the full 16 MB. -->
      <SocketReceiveBufferSize min="16MB"/>
      <SocketSendBufferSize min="2MB"/>
    </Internal>
  </Domain>
</CycloneDDS>
```

Adjust for your machine:

- `NetworkInterface name` — your interface on the 192.168.0.0/24 network
  (`ip -4 addr` to find it).
- `Peer address` — the robot's current IP. If the robot gets its address
  from DHCP and moves, discovery breaks exactly as described above; give it
  a static lease on the AP, or update this entry when it moves.
- `Domain Id` must match `ROS_DOMAIN_ID` (42 in `qb3rt_env.sh`) and the
  robot's config.

The config is read once at node startup — after editing it, restart any
running nodes (RViz, Nav2, and the CLI daemon if you use one) or they keep
the old peer list.

## Clock sync (do not skip)

TF is stamped by the robot and consumed on the host. If the clocks disagree
by more than the transform tolerances (~0.3-0.5 s), every costmap update and
controller cycle fails with extrapolation errors. Check:

```bash
# on the host
date +%s.%N; ssh/console on robot: date +%s.%N   # or compare `ros2 topic echo /scan --field header.stamp`
```

If skewed, sync the RB3 (chrony/NTP against the router or the host) before
launching. Symptoms of skew: "Lookup would require extrapolation into the
future/past" spam from costmap_2d / RPP.

## Run order

1. **Robot** (in your device shell):

   ```bash
   source /root/rover_env.sh
   ros2 launch QB3rt full_stack.launch.py enable_nav:=false
   ```

   Wait until: `/scan` streaming, `/map` publishing (slam_toolbox up),
   `/odometry/filtered` streaming (EKF up). ORB-SLAM3 needs a little
   translation to initialize VIO; the EKF runs fine before that on IMU+wheel vx.

2. **Host**:

   ```bash
   source ~/qb3rt_host/qb3rt_env.sh
   ros2 launch ~/qb3rt_host/nav2_host.launch.py
   ```

   RViz opens with the Nav2 default view. Check the map and TF arrive, then
   send a goal with "Nav2 Goal".

## Sanity checks when something is off

```bash
source ~/qb3rt_host/qb3rt_env.sh
ros2 topic hz /scan                # lidar arriving over WiFi?
ros2 topic hz /odometry/filtered   # EKF arriving?
ros2 run tf2_tools view_frames     # map->odom->base_footprint->base_link chain complete?
ros2 topic echo /cmd_vel --once    # controller output reaching DDS?
```

- No topics at all -> env not sourced / wrong `CYCLONEDDS_URI` (must be
  `~/cyclonedds.xml`, **not** `/opt/cyclonedds.xml` — that path only exists on
  the robot) / robot's IP missing from `<Peers>` (see
  [CycloneDDS setup](#cyclonedds-setup)) / robot not up.
- Topics but TF errors -> clock skew (above).
- Goal accepted but robot doesn't move -> is the bridge running with
  `enable_base_driver:=true` (full_stack default)? `ros2 topic hz /cmd_vel` on
  the robot side.

## Notes on this config

- Planner is **SmacPlannerHybrid, REEDS_SHEPP model, minimum_turning_radius
  0.35** — the skid-steer cannot pivot, but REEDS_SHEPP permits a short
  reverse instead of the wide forward-only loop DUBIN forces when a goal's
  heading mismatch has no forward-only solution (changed 2026-07-14; DUBIN
  caused 13 consecutive circling replans on one live goal). Paths can include
  brief reversing; that is intentional.
- Controller is Regulated Pure Pursuit with `use_rotate_to_heading: false`,
  loose `yaw_goal_tolerance` (base can't fine-tune heading in place), and
  `allow_reversing: true` to match the planner. `reverse_penalty: 4.0`
  reserves reversing for clearly-shorter cases (raised from 2.0 after one
  goal oscillated 28s at a Reeds-Shepp cusp) and `movement_time_allowance:
  8.0` (down from 15.0) aborts a stuck/oscillating robot roughly 2x faster.
  `regulated_linear_scaling_min_radius: 0.5` — a soft speed-reduction
  threshold, kept slightly above `minimum_turning_radius` as tracking
  margin; do not set planner/controller turn-radius values from open-loop
  drive calibration numbers, they measure feedforward accuracy, not a
  kinematic limit (bit us 2026-07-14, see qb3rt-restructure memory).
- Reversing is tuned and works on most goals, but is not exhaustively
  road-tested — watch new reversing maneuvers, especially near cusps.
- Recovery behaviors exclude Spin (the custom `*_no_spin.xml` BTs).
- `transform_tolerance` is raised vs the onboard config to absorb WiFi latency.
- VIO is NOT fused into the EKF (wheel vx + gyro yaw rate only) — don't wait
  on VIO init before launching Nav2, and don't expect VIO issues to affect
  navigation.

## License

MIT — see [LICENSE](LICENSE).
