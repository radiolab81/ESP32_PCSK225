idf_component_register(
    SRCS
        "main.cpp"
        "pl225/pl225_decoder.cpp"
    INCLUDE_DIRS
        "."
        "pl225"
    REQUIRES
        esp_adc
        esp_timer
        driver
)

# Der Dekoder-Code ist C++17 (std::complex, std::deque, structured-ish
# Nutzung von <cmath>), im Referenzprojekt ohnehin schon C++17.
target_compile_features(${COMPONENT_LIB} PRIVATE cxx_std_17)
