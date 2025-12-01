idf_component_register(SRCS "main.c"
                       INCLUDE_DIRS "."
                       )

# ★★★ 新增這一行 ★★★
# 這會把 index.html 轉換成二進位數據嵌入到韌體中
# 編譯後會自動產生 _binary_index_html_start 和 _binary_index_html_end 符號供 C 語言呼叫
target_add_binary_data(${COMPONENT_TARGET} "index.html" TEXT)