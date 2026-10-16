cmake_minimum_required(VERSION 3.8)
project(go2_patrol_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

# maps/ 特殊处理：地图是运行期（save_map.sh）才生成的，而 symlink-install
# 只会为“构建时就存在”的文件建链接，所以不能放进上面的 install(DIRECTORY)。
# 这里在 install 阶段直接把整个 maps/ 目录做成符号链接，使
# install/share/<pkg>/maps 与 src/go2_patrol_bringup/maps 始终是同一份内容。
# 非 symlink-install 构建下退化为普通目录复制。
#
# 注意：AMENT_CMAKE_SYMLINK_INSTALL 是 CMake option（配置期变量），
# 在 install 阶段不可见，因此这里在配置期把它固化成字面值再插进脚本。
if(AMENT_CMAKE_SYMLINK_INSTALL)
  set(_maps_symlink_branch "1")
else()
  set(_maps_symlink_branch "0")
endif()
install(CODE "
  set(_maps_src \"${CMAKE_CURRENT_SOURCE_DIR}/maps\")
  set(_maps_dst \"\${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/maps\")
  file(MAKE_DIRECTORY \"\${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}\")
  file(REMOVE_RECURSE \"\${_maps_dst}\")
  if(${_maps_symlink_branch})
    execute_process(COMMAND \${CMAKE_COMMAND} -E create_symlink \"\${_maps_src}\" \"\${_maps_dst}\")
  else()
    file(COPY \"\${_maps_src}\" DESTINATION \"\${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}\")
  endif()
")

ament_package()
