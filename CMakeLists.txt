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

find_library(MUJOCO_LIBRARY
    NAMES mujoco mujoco.3.8.1
    PATHS
        ${MUJOCO_ROOT}
        ${MUJOCO_ROOT}/lib
        ${MUJOCO_ROOT}/Libraries
        /usr/local/lib
)

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

# ── インストール ─────────────────────────────────────────────────────
install(TARGETS spikehat_sim
    LIBRARY DESTINATION lib
)
install(FILES include/spikehat.h
    DESTINATION include
)
