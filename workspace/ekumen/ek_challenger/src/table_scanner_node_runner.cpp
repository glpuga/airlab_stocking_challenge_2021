/* Copyright [2021] <Ekumen>
 * Author: Gerardo Puga
 */

// project
#include <ek_challenger/table_scanner_node.hpp>

int main(int argc, char **argv) {
  ros::init(argc, argv, "table_scanner_node");
  ek_challenger::TableScannerNode bt;
  return bt.run() ? 0 : -1;
}
