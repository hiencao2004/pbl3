# Tên thư mục component (ds18b20) sẽ được sử dụng làm tên component
# Đăng ký các file nguồn (.c) cho component này
idf_component_register(SRCS "ds18b20.c" 
                       INCLUDE_DIRS "."  # <--- DÒNG QUAN TRỌNG NHẤT
                    PRIV_REQUIRES
                       esp_driver_gpio
)