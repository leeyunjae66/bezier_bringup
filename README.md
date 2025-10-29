# UROP Bezier Bringup

## 의존성
- ROS 2 (Humble 등)
- nav2_map_server, nav2_lifecycle_manager, rviz2

### 실행 방법
```bash
ros2 launch bezier_bringup bringup.launch.py

#### bringup 코드 사용 설명
-코드 실행후 rviz2 gui로 publish points 4개를 클릭.
-cp를 충분한 안전거리로 밀기때문에 보정된 path가 밀린 방향의 장애물과 충돌할 수 있음.
