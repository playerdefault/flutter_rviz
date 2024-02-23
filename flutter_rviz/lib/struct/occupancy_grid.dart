import './pose.dart';

class Stamp {
  int nanosec;
  int sec;

  Stamp({required this.nanosec, required this.sec});
}

class Header {
  String frameId;
  Stamp stamp;

  Header({required this.frameId, required this.stamp});
}

class MapMetaData {
  double resolution;
  int width;
  int height;
  Pose origin;

  MapMetaData({required this.resolution, required this.width, required this.height, required this.origin});
}

class OccupancyGrid {
  Header header;
  MapMetaData mapMetaData;
  List<int> data;

  OccupancyGrid({required this.header, required this.mapMetaData, required this.data});

  // Factory constructor for creating an instance from a decoded JSON response
  factory OccupancyGrid.fromJson(Map<String, dynamic> response) {
    List<dynamic> dataDynamicList = response["data"];
    List<int> dataIntList = dataDynamicList.map<int>((item) => item as int).toList();

    return OccupancyGrid(
      header: Header(
        frameId: response["header"]["frame_id"],
        stamp: Stamp(
          nanosec: response["header"]["stamp"]["nanosec"],
          sec: response["header"]["stamp"]["sec"],
        ),
      ),
      mapMetaData: MapMetaData(
        resolution: response["info"]["resolution"],
        width: response["info"]["width"],
        height: response["info"]["height"],
        origin: Pose(
          position: Position(
            x: response["info"]["origin"]["position"]["x"],
            y: response["info"]["origin"]["position"]["y"],
            z: response["info"]["origin"]["position"]["z"],
          ),
          orientation: Orientation(
            x: response["info"]["origin"]["orientation"]["x"],
            y: response["info"]["origin"]["orientation"]["y"],
            z: response["info"]["origin"]["orientation"]["z"],
            w: response["info"]["origin"]["orientation"]["w"],
          ),
        ),
      ),
      data: dataIntList,
    );
  }
}
