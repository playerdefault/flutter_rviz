import 'package:flutter_test/flutter_test.dart';
import "dart:convert";
import "dart:async";
import "package:web_socket_channel/web_socket_channel.dart";
import "package:flutter_rviz/struct/occupancy_grid.dart";
import "package:flutter_rviz/utils/og_to_image.dart";

import 'package:flutter_rviz/flutter_rviz.dart';

Future<OccupancyGrid> callROSServiceAndGetOG() async {
  final channel = WebSocketChannel.connect(Uri.parse("ws://localhost:9090"));

  final serviceCallMessage =
      jsonEncode({"op": "call_service", "service": "/map_server/map"});

  channel.sink.add(serviceCallMessage);

  final completer = Completer<void>();
  late OccupancyGrid result;

  channel.stream.listen((message) {
    result = OccupancyGrid.fromJson(jsonDecode(message)["values"]["map"]);

    if (!completer.isCompleted) {
      completer.complete();
    }
  }, onDone: () {
    print("WebSocket connection closed.");
    if (!completer.isCompleted) {
      completer.complete();
    }
  }, onError: (error) {
    print("WebSocket Error: $error");
    if (!completer.isCompleted) {
      completer.completeError(error);
    }
  });

  // Wait for the stream listener to complete.
  await completer.future;

  // Optionally, close the connection explicitly here.
  channel.sink.close();
  return result;
}

void main() {
  // PREREQUISITES: ros2 humble should be installed on your system. You also need to install the necessary packages inside
  // the ros2_ws. Check the CMakeLists.txt inside test/ros2_ws/src/map_pub for what packages are needed.

  // IMPORTANT: Make sure to navigate to ros2_ws inside the test folder, source the install space,
  // and execute the launch file. Relevant commands:
  // cd ros2_ws
  // colcon build (wait for the build to finish)
  // source install/setup.bash
  // ros2 launch map_pub main.launch.py

  // Above instructions need to be executed to test this properly.

  test("call map service via ros websockets", () async {
    final og = await callROSServiceAndGetOG();
    final pngImgData = occupancyGridToImageBytes(og);

    // Assert that og is not null
    expect(og, isNotNull);
  });
}
