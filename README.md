# UROP Bezier Bringup

## 의존성
- ROS 2 (Humble 등)
- nav2_map_server, nav2_lifecycle_manager, rviz2

#### Bringup 코드 사용 설명

- 코드를 실행한 후 RViz2 GUI에서 `Publish Point` 도구를 이용해 4개의 제어점(Control Points) 을 클릭합니다.  
- 본 알고리즘은 제어점을 충분한 안전거리로 외측 이동시켜 곡선을 보정합니다.  
  단, 이 과정에서 보정된 경로가 밀린 방향의 장애물과 충돌할 가능성이 있습니다.


### 실행 방법
```bash
ros2 launch bezier_bringup bringup.launch.py
