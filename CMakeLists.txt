cmake_minimum_required(VERSION 3.16)
project(libspikehat_sim C)

set(CMAKE_C_STANDARD 11)

# ── MuJoCo の検索 ────────────────────────────────────────────────────
# MUJOCO_ROOT 環境変数またはデフォルトパスから探す
if(DEFINED ENV{MUJOCO_ROOT})
    set(MUJOCO_ROOT $ENV{MUJOCO_ROOT})
else()
    # macOS のデフォルト（MuJoCo.app からのインストール）
    set(MUJOCO_ROOT "/Applications/MuJoCo.app/Contents/Frameworks/MuJoCo.framework/Versions/Current")
    if(NOT EXISTS ${MUJOCO_ROOT})
        # pip install mujoco でインストールした場合
        execute_process(
            COMMAND python3 -c "import mujoco; import os; print(os.path.dirname(mujoco.__file__))"
            OUTPUT_VARIABLE MUJOCO_PYTHON_DIR
            OUTPUT_STRIP_TRAILING_WHITESPACE
        )
        set(MUJOCO_ROOT ${MUJOCO_PYTHON_DIR})
    endif()
endif()

message(STATUS "MUJOCO_ROOT: ${MUJOCO_ROOT}")

find_path(MUJOCO_INCLUDE_DIR mujoco/mujoco.h
    PATHS
        ${MUJOCO_ROOT}/include
        ${MUJOCO_ROOT}/Headers
        /usr/local/include
)

file(GLOB _MUJOCO_DYLIBS
    "${MUJOCO_ROOT}/libmujoco*.dylib"
    "${MUJOCO_ROOT}/lib/libmujoco*.dylib"
    "/usr/local/lib/libmujoco*.dylib"
)
list(GET _MUJOCO_DYLIBS 0 MUJOCO_LIBRARY)
if(NOT MUJOCO_LIBRARY)
    find_library(MUJOCO_LIBRARY
        NAMES mujoco
        PATHS ${MUJOCO_ROOT}/lib /usr/local/lib
        NO_DEFAULT_PATH
    )
endif()

if(NOT MUJOCO_INCLUDE_DIR OR NOT MUJOCO_LIBRARY)
    message(FATAL_ERROR
        "MuJoCo が見つかりません。\n"
        "MUJOCO_ROOT 環境変数でパスを指定してください:\n"
        "  cmake -DMUJOCO_ROOT=/path/to/mujoco ..")
endif()

message(STATUS "MuJoCo include: ${MUJOCO_INCLUDE_DIR}")
message(STATUS "MuJoCo library: ${MUJOCO_LIBRARY}")

# ── libspikehat_sim 共有ライブラリ ───────────────────────────────────
add_library(spikehat_sim SHARED
    src/sim_spikehat.c
)

target_include_directories(spikehat_sim
    PUBLIC
        ${CMAKE_SOURCE_DIR}/include
        ${MUJOCO_INCLUDE_DIR}
)

target_link_libraries(spikehat_sim
    PUBLIC
        ${MUJOCO_LIBRARY}
        pthread
        m
)

# macOS では RPATH を設定
if(APPLE)
    set_target_properties(spikehat_sim PROPERTIES
        INSTALL_RPATH "@loader_path"
        BUILD_RPATH   "${MUJOCO_ROOT}/lib"
    )
endif()

# ── test_motor（C版） ────────────────────────────────────────────────
add_executable(test_motor
    examples/test_motor.c
)

target_include_directories(test_motor
    PRIVATE
        ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(test_motor
    PRIVATE
        spikehat_sim
)

# ── test_color_sensor（C版） ─────────────────────────────────────────
add_executable(test_color_sensor
    examples/test_color_sensor.c
)

target_include_directories(test_color_sensor
    PRIVATE
        ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(test_color_sensor
    PRIVATE
        spikehat_sim
)

# ── test_distance_sensor（C版） ──────────────────────────────────────
add_executable(test_distance_sensor
    examples/test_distance_sensor.c
)

target_include_directories(test_distance_sensor
    PRIVATE
        ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(test_distance_sensor
    PRIVATE
        spikehat_sim
)

# ── test_force_sensor（C版） ──────────────────────────────────────────
add_executable(test_force_sensor
    examples/test_force_sensor.c
)

target_include_directories(test_force_sensor
    PRIVATE
        ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(test_force_sensor
    PRIVATE
        spikehat_sim
)

# ── インストール ─────────────────────────────────────────────────────
install(TARGETS spikehat_sim
    LIBRARY DESTINATION lib
)
install(FILES include/spikehat.h
    DESTINATION include
)

# ── macOS: mujoco.framework シンボリックリンクのセットアップ ─────────
if(APPLE)
    # MUJOCO_ROOT はすでに検出済みのパスを使う
    set(MUJOCO_PY_DIR ${MUJOCO_ROOT})
    # dylib のバージョンを動的に取得
    file(GLOB _MUJOCO_DYLIB_FILES "${MUJOCO_PY_DIR}/libmujoco.*.dylib")
    list(GET _MUJOCO_DYLIB_FILES 0 _DYLIB)
    get_filename_component(_DYLIB_NAME "${_DYLIB}" NAME)
    set(_FWDIR "${MUJOCO_PY_DIR}/mujoco.framework/Versions/A")
    file(MAKE_DIRECTORY ${_FWDIR})
    execute_process(
        COMMAND ${CMAKE_COMMAND} -E create_symlink
            ${_DYLIB}
            "${_FWDIR}/${_DYLIB_NAME}"
    )
    message(STATUS "mujoco.framework symlink: ${_FWDIR}/${_DYLIB_NAME}")
endif()
