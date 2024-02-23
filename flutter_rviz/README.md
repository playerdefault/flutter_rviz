<!--
This README describes the package. If you publish this package to pub.dev,
this README's contents appear on the landing page for your package.

For information about how to write a good package README, see the guide for
[writing package pages](https://dart.dev/guides/libraries/writing-package-pages).

For general information about developing packages, see the Dart guide for
[creating packages](https://dart.dev/guides/libraries/create-library-packages)
and the Flutter guide for
[developing packages and plugins](https://flutter.dev/developing-packages).
-->

# flutter_rviz

Mini RViz in Flutter for showing basic ROS data.

Converts Occupancy Grid obtained by ROS 2 websockets to custom OccupancyGrid object and converts OccupancyGrid to a Uint8Bytes image object.

Tested with ROS 2 Humble as of today.

## Installation

Add it to your flutter package by
```bash
flutter pub add flutter_rviz
```

## Usage

First create an OccupancyGrid Object and populate values in all the fields.
The convert that OccupancyGrid object to Uint8Bytes PNG via `occupancyGridToImageBytes`.
Then create a Flutter Image object and use as required: `Image.memory(bytesObjCreatedAbove)`

## Contributing

Pull requests are welcome. For major changes, please open an issue first
to discuss what you would like to change.

Please make sure to update tests as appropriate.

## License

[MIT](https://choosealicense.com/licenses/mit/)