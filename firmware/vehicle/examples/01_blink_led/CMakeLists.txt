cmake_minimum_required(VERSION 3.16)
set(EXTRA_COMPONENT_DIRS ../../components)
# Build only main and what it REQUIRES; otherwise every component under
# ../../components is pulled in, and their dependencies must all resolve.
# main とその REQUIRES だけを組む。指定しないと ../../components の全部品が
# 取り込まれ、その依存も全て解決できなければならなくなる。
set(COMPONENTS main)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(01_blink_led)
