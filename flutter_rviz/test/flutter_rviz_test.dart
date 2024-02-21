import 'package:flutter_test/flutter_test.dart';
import "dart:convert";
import "dart:async";
import "package:web_socket_channel/web_socket_channel.dart";
import "package:web_socket_channel/status.dart" as status;
import "package:flutter_rviz/struct/occupancy_grid.dart";

import 'package:flutter_rviz/flutter_rviz.dart';

Future callROSService() async {
  final channel = WebSocketChannel.connect(Uri.parse("ws://localhost:9090"));

  final serviceCallMessage = jsonEncode({
    "op": "call_service",
    "service": "/map_server/map"
  });

  channel.sink.add(serviceCallMessage);

  final completer = Completer<void>();

  OccupancyGrid og;

  channel.stream.listen((message) {
    final response = jsonDecode(message);

    // Printing occupancy grid
    print(response["values"]["map"]["data"]);

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
}

void main() {
  test("call map service and print output", () async {
    await callROSService();
  });
}