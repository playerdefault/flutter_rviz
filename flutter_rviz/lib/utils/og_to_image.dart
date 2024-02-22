import 'dart:typed_data';

import 'package:image/image.dart' as img;
import 'package:flutter_rviz/struct/occupancy_grid.dart';

Uint8List occupancyGridToImageBytes(OccupancyGrid og) {
  final result =
      img.Image(width: og.mapMetaData.width, height: og.mapMetaData.height);

  for (int i = 0; i < og.data.length; i++) {
    final x = i % og.mapMetaData.width;
    final y = (i / og.mapMetaData.width).floor();

    if (og.data[i] == 1) {
      // Occupied Cell: Black Color
      result.setPixelRgb(x, y, 0, 0, 0);
    } else if (og.data[i] == 0) {
      // Free Cell: White Color
      result.setPixelRgb(x, y, 255, 255, 255);
    } else if (og.data[i] == -1) {
      // Unknown Cell Occupancy: Grey Color
      result.setPixelRgb(x, y, 211, 211, 211);
    }
  }

  return img.encodePng(result);
}
