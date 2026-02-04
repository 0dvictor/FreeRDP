# FreeRDP: A Remote Desktop Protocol Implementation
# FreeRDP DRM Client cmake build script
#
# Copyright 2026 Victor Ding <victor@ding.pm>
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

set(MODULE_NAME "drm-freerdp")
set(MODULE_PREFIX "FREERDP_CLIENT_DRM")
project(${MODULE_NAME} LANGUAGES CXX)

find_package(PkgConfig REQUIRED)
pkg_check_modules(UDEV REQUIRED IMPORTED_TARGET libudev)

set(${MODULE_PREFIX}_SRCS main.cpp)

list(APPEND ${MODULE_PREFIX}_LIBS freerdp-client freerdp PkgConfig::UDEV)

addtargetwithresourcefile(${MODULE_NAME} TRUE ${FREERDP_VERSION} ${MODULE_PREFIX}_SRCS)

target_link_libraries(${MODULE_NAME} ${${MODULE_PREFIX}_LIBS})

install(TARGETS ${MODULE_NAME} DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT client)

set_property(TARGET ${MODULE_NAME} PROPERTY FOLDER "Client/DRM")
