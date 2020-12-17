#!/usr/bin/env bats

@test "stitch basic panorama" {
  input_images=$(find ../test/fixtures/panorama_aus_1 -name *.JPG | xargs)
  output=$(./airmap_stitcher $input_images)
  [ "$output" = "[stitcher]>Written stitched image to ./panorama.jpg" ]
  panorama_stats=$(stat -c "%s %n" panorama.jpg)
  [ "$panorama_stats" = "3084883 panorama.jpg" ]
}

