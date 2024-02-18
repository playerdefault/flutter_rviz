import './pose.dart';

class Header {
  String frameId;
  DateTime stamp;

  Header({required this.frameId, required this.stamp});
}

class MapMetaData {
  double resolution;
  int width;
  int height;
  Pose origin;

  MapMetaData(
      {required this.resolution,
      required this.width,
      required this.height,
      required this.origin});
}

class OccupancyGrid {
  Header header;
  MapMetaData mapMetaData;
  List<int> data;

  OccupancyGrid(
      {required this.header, required this.mapMetaData, required this.data});
}
