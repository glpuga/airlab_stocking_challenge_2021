/* Copyright [2021] <Ekumen>
 * Author: Gerardo Puga
 */

// project
#include <ek_challenger/shelf_scanner_node.hpp>

int main(int argc, char **argv) {
  ros::init(argc, argv, "shelf_scanner_node");
  ek_challenger::ShelfScannerNode bt;
  return bt.run() ? 0 : -1;
}
