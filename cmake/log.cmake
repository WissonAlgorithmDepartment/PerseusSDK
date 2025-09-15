# ============================
# cmake/log.cmake
# ============================

include_guard(GLOBAL)

# 检测是否支持 ANSI 颜色
# Windows cmd 默认不支持，需要通过 CMake 判断
if(WIN32 AND NOT (CMAKE_GENERATOR STREQUAL "MinGW Makefiles" OR CMAKE_GENERATOR STREQUAL "Ninja"))
    set(PERSEUS_ENABLE_COLOR OFF)
else()
    # 非 Windows 系统或支持颜色的 Windows 终端默认启用
    set(PERSEUS_ENABLE_COLOR ON)
endif()

# 如果用户手动覆盖
if(DEFINED ENABLE_COLOR_OUTPUT)
    set(PERSEUS_ENABLE_COLOR ${ENABLE_COLOR_OUTPUT})
endif()

# 定义 ANSI 颜色序列
if(PERSEUS_ENABLE_COLOR)
    string(ASCII 27 Esc)
    set(RESET   "${Esc}[0m")
    set(RED     "${Esc}[1;31m")
    set(GREEN   "${Esc}[1;32m")
    set(YELLOW  "${Esc}[1;33m")
    set(BLUE    "${Esc}[1;34m")
    set(CYAN    "${Esc}[1;36m")
else()
    set(RESET   "")
    set(RED     "")
    set(GREEN   "")
    set(YELLOW  "")
    set(BLUE    "")
    set(CYAN    "")
endif()

# ---- 内部辅助函数: 布尔值上色 ----
function(_colorize_bool val return_var)
    if(NOT DEFINED val OR val STREQUAL "")
        set(val "<unset>")
    endif()
    string(TOUPPER "${val}" val_upper)
    if(val_upper STREQUAL "ON" OR val_upper STREQUAL "TRUE" OR val_upper STREQUAL "YES")
        set(${return_var} "${GREEN}${val}${RESET}" PARENT_SCOPE)
    elseif(val_upper STREQUAL "OFF" OR val_upper STREQUAL "FALSE" OR val_upper STREQUAL "NO")
        set(${return_var} "${RED}${val}${RESET}" PARENT_SCOPE)
    else()
        set(${return_var} "${val}" PARENT_SCOPE)
    endif()
endfunction()

# ---- 内部辅助函数: 处理消息中的布尔值 ----
function(_process_message msg return_var)
    # 将消息拆分为单词
    string(REGEX MATCHALL "[^ ]+" words "${msg}")
    
    set(colored_msg "")
    foreach(word IN LISTS words)
        # 检查是否为布尔值
        _colorize_bool("${word}" colored_word)
        # 重新构建消息
        if(colored_msg STREQUAL "")
            set(colored_msg "${colored_word}")
        else()
            set(colored_msg "${colored_msg} ${colored_word}")
        endif()
    endforeach()
    
    set(${return_var} "${colored_msg}" PARENT_SCOPE)
endfunction()

# ---- 封装打印函数 ----
function(perseus_print msg)
    message(STATUS "${BLUE}[INFO]${RESET} ${msg}")
endfunction()

function(perseus_info msg val)
    _colorize_bool("${val}" colored_val)
    message(STATUS "${BLUE}[INFO]${RESET} ${msg}${colored_val}")
endfunction()

function(perseus_success msg)
    message(STATUS "${GREEN}[SUCC]${RESET} ${msg}")
endfunction()

function(perseus_warn msg val)
    message(STATUS "${YELLOW}[WARN]${RESET} ${msg}${YELLOW}${val}${RESET}")
endfunction()

function(perseus_error msg)
    message(FATAL_ERROR "${RED}[ERROR]${RESET} ${msg}")
endfunction()

# ---- 打印键值对（带布尔值颜色） ----
function(perseus_info_key_value key value)
    _colorize_bool("${value}" colored_value)
    message(STATUS "${CYAN}[INFO]${RESET} ${key}: ${colored_value}")
endfunction()

# ---- 打印空行 ----
function(perseus_newline)
    message(STATUS "")
endfunction()

function(perseus_print_separator length)
    if(NOT length)
        set(length 90)  # 默认长度
    endif()
    string(REPEAT "=" ${length} LINE)
    #perseus_print("${LINE}")
    message(STATUS "${LINE}")
endfunction()

function(perseus_print_top_separator)
    perseus_print_separator(90)
endfunction()

function(perseus_print_sub_separator)
    perseus_print_separator(90) 
    perseus_newline()
    perseus_newline()
endfunction()

function(perseus_print_sub_separator_start)
    perseus_print_separator(90) 
endfunction()

function(perseus_print_fixed_header text)
    set(total_length 70)

    if("${text}" STREQUAL "")
        string(REPEAT "-" ${total_length} line)
        perseus_print("${line}")
        #message(STATUS "${line}")
        return()
    endif()

    set(text " ${text} ")
    
    string(LENGTH "${text}" text_length)
    math(EXPR padding "( ${total_length} - ${text_length} ) / 2")
    if(padding LESS 0)
        set(padding 0)
    endif()
    string(REPEAT "-" ${padding} pad_left)
    string(REPEAT "-" ${padding} pad_right)
    # 如果总长度是奇数，右边多一个 '-' 来凑齐总长度
    math(EXPR extra " ${total_length} - ${text_length} - ${padding} * 2 ")
    if(extra GREATER 0)
        set(pad_right "${pad_right}-")
    endif()
    perseus_print("${pad_left}${text}${pad_right}")
    #message(STATUS "${pad_left}${text}${pad_right}")
endfunction()