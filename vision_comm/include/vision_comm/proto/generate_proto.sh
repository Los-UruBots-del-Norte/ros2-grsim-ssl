#!/bin/bash

# Specify the source directory containing .proto files
PROTO_SOURCE_DIR="./"

# Specify the destination directory to copy the generated files
DESTINATION_DIR="../"

# Generate .proto files
protoc --cpp_out=. messages_robocup_ssl_detection.proto
protoc --cpp_out=. messages_robocup_ssl_geometry.proto
protoc --cpp_out=. messages_robocup_ssl_refbox_log.proto
protoc --cpp_out=. messages_robocup_ssl_wrapper.proto

# Copy the generated files to the destination folder
cp $PROTO_SOURCE_DIR/*.{pb.cc,pb.h} $DESTINATION_DIR

echo "Proto files generated and copied to $DESTINATION_DIR"