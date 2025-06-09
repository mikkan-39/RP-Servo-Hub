add_library(pico_ssd1306
        ssd1306.cpp
        frameBuffer/FrameBuffer.cpp
        shapeRenderer/ShapeRenderer.cpp)

add_subdirectory(textRenderer)

target_link_libraries(pico_ssd1306
        ssd1306_textRenderer
        pico_stdlib
        hardware_spi
        )

target_include_directories (pico_ssd1306 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})