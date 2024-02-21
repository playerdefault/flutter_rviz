class Position {
  double x;
  double y;
  double z;

  Position({required this.x, required this.y, required this.z});
}

class Orientation {
  double x;
  double y;
  double z;
  double w;

  Orientation(
      {required this.x, required this.y, required this.z, required this.w});
}

class Pose {
  Position position;
  Orientation orientation;

  Pose({required this.position, required this.orientation});
}
