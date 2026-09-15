# Copyright (c) 2026, BlackBerry Limited. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

include_guard(GLOBAL)

include(CheckCCompilerFlag)
include(CheckCXXCompilerFlag)
include(CheckLinkerFlag)

function(_baby_robot_add_checked_compile_option target language option)
  string(MAKE_C_IDENTIFIER "${language}_${option}" option_id)
  set(result_var "BABY_ROBOT_SUPPORTS_COMPILE_${option_id}")

  if(language STREQUAL "C")
    check_c_compiler_flag("${option}" "${result_var}")
  elseif(language STREQUAL "CXX")
    check_cxx_compiler_flag("${option}" "${result_var}")
  else()
    message(FATAL_ERROR "Unsupported hardening language '${language}' for ${target}")
  endif()

  if(${result_var})
    # Mixed-language targets must use only options accepted by the compiler
    # for the source currently being compiled.
    target_compile_options(
      ${target} PRIVATE "$<$<COMPILE_LANGUAGE:${language}>:${option}>")
  else()
    message(WARNING
      "${target}: ${language} compiler does not support hardening option '${option}'")
  endif()
endfunction()

function(_baby_robot_add_checked_link_option target language option)
  string(MAKE_C_IDENTIFIER "${language}_${option}" option_id)
  set(result_var "BABY_ROBOT_SUPPORTS_LINK_${option_id}")
  check_linker_flag("${language}" "${option}" "${result_var}")

  if(${result_var})
    target_link_options(${target} PRIVATE "${option}")
  else()
    message(WARNING
      "${target}: ${language} linker does not support hardening option '${option}'")
  endif()
endfunction()

# Add deployable-executable hardening only after both compiler front ends and
# the executable's actual linker driver accept each option. Fortify also needs
# an optimized build; build.sh therefore pins RelWithDebInfo.
function(baby_robot_enable_checked_hardening target)
  set(options)
  set(one_value_args LINK_LANGUAGE)
  set(multi_value_args LANGUAGES)
  cmake_parse_arguments(
    HARDEN "${options}" "${one_value_args}" "${multi_value_args}" ${ARGN})

  if(HARDEN_UNPARSED_ARGUMENTS OR HARDEN_KEYWORDS_MISSING_VALUES)
    message(FATAL_ERROR "Invalid hardening arguments for ${target}")
  endif()
  if(NOT TARGET ${target})
    message(FATAL_ERROR "Cannot harden missing target '${target}'")
  endif()
  if(NOT HARDEN_LANGUAGES OR NOT HARDEN_LINK_LANGUAGE)
    message(FATAL_ERROR
      "Hardening ${target} requires LANGUAGES and LINK_LANGUAGE")
  endif()

  get_target_property(target_type ${target} TYPE)
  if(NOT target_type STREQUAL "EXECUTABLE")
    message(FATAL_ERROR "Checked PIE hardening requires an executable target: ${target}")
  endif()

  foreach(language IN LISTS HARDEN_LANGUAGES)
    if(NOT CMAKE_${language}_COMPILER_LOADED)
      message(FATAL_ERROR "Language ${language} is not enabled for ${target}")
    endif()
    _baby_robot_add_checked_compile_option(
      ${target} "${language}" "-fstack-protector-strong")
    _baby_robot_add_checked_compile_option(
      ${target} "${language}" "-D_FORTIFY_SOURCE=2")
    _baby_robot_add_checked_compile_option(${target} "${language}" "-fPIE")
  endforeach()

  _baby_robot_add_checked_link_option(${target} "${HARDEN_LINK_LANGUAGE}" "-pie")
  _baby_robot_add_checked_link_option(
    ${target} "${HARDEN_LINK_LANGUAGE}" "-Wl,-z,relro")
  _baby_robot_add_checked_link_option(
    ${target} "${HARDEN_LINK_LANGUAGE}" "-Wl,-z,now")

  if(QNX)
    message(STATUS
      "${target}: target release gate must inspect the QNX ELF for PIE, "
      "RELRO/NOW, stack-canary, and effective fortify evidence")
  endif()
endfunction()

# Target residual: CFI/LTO are intentionally not guessed here. They require a
# qualified QNX compiler/runtime combination and compatible instrumentation of
# linked ROS dependencies. Release validation must also inspect the final QNX
# ELF because successful driver probes do not prove that every property survived
# the target linker and post-processing pipeline.
