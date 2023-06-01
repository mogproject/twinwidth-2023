#pragma once
#include <chrono>
#include <cstdio>

namespace util {
namespace logging {
int const TRACE = 5;
int const DEBUG = 10;
int const INFO = 20;
int const WARNING = 30;
int const ERROR = 40;
int const CRITICAL = 50;
int const SUCCESS = 60;
int const NONE = 100;
extern int log_level;
}  // namespace logging

void print_trace_prefix();
void print_debug_prefix();
void print_info_prefix();
void print_warning_prefix();
void print_error_prefix();
void print_critical_prefix();
void print_success_prefix();

void print_trace_postfix();
void print_debug_postfix();
void print_info_postfix();
void print_warning_postfix();
void print_error_postfix();
void print_critical_postfix();
void print_success_postfix();

void set_log_level(int log_level);

void timer_start(int key = 0);
double timer_stop(int key = 0);
}  // namespace util

//==================================================================================================
// Logging macros
//==================================================================================================
// clang-format off
#if LOGGING_ON
#define log_trace(format, ...) {if (util::logging::TRACE >= util::logging::log_level) {util::print_trace_prefix();fprintf(stderr, format, __VA_ARGS__);util::print_trace_postfix();}}
#define log_trace_(format) {if (util::logging::TRACE >= util::logging::log_level) {util::print_trace_prefix();fprintf(stderr, format);util::print_trace_postfix();}}
#define log_debug(format, ...) {if (util::logging::DEBUG >= util::logging::log_level) {util::print_debug_prefix();fprintf(stderr, format, __VA_ARGS__);util::print_debug_postfix();}}
#define log_debug_(format) {if (util::logging::DEBUG >= util::logging::log_level) {util::print_debug_prefix();fprintf(stderr, format);util::print_debug_postfix();}}
#define log_info(format, ...) {if (util::logging::INFO >= util::logging::log_level) {util::print_info_prefix();fprintf(stderr, format, __VA_ARGS__);util::print_info_postfix();}}
#define log_info_(format) {if (util::logging::INFO >= util::logging::log_level) {util::print_info_prefix();fprintf(stderr, format);util::print_info_postfix();}}
#define log_warning(format, ...) {if (util::logging::WARNING >= util::logging::log_level) {util::print_warning_prefix();fprintf(stderr, format, __VA_ARGS__);util::print_warning_postfix();}}
#define log_warning_(format) {if (util::logging::WARNING >= util::logging::log_level) {util::print_warning_prefix();fprintf(stderr, format);util::print_warning_postfix();}}
#define log_error(format, ...) {if (util::logging::ERROR >= util::logging::log_level) {util::print_error_prefix();fprintf(stderr, format, __VA_ARGS__);util::print_error_postfix();}}
#define log_error_(format) {if (util::logging::ERROR >= util::logging::log_level) {util::print_error_prefix();fprintf(stderr, format);util::print_error_postfix();}}
#define log_critical(format, ...) {if (util::logging::CRITICAL >= util::logging::log_level) {util::print_critical_prefix();fprintf(stderr, format, __VA_ARGS__);util::print_critical_postfix();}}
#define log_critical_(format) {if (util::logging::CRITICAL >= util::logging::log_level) {util::print_critical_prefix();fprintf(stderr, format);util::print_critical_postfix();}}
#define log_success(format, ...) {if (util::logging::SUCCESS >= util::logging::log_level) {util::print_success_prefix();fprintf(stderr, format, __VA_ARGS__);util::print_success_postfix();}}
#define log_success_(format) {if (util::logging::SUCCESS >= util::logging::log_level) {util::print_success_prefix();fprintf(stderr, format);util::print_success_postfix();}}
#else
#define log_trace(format, ...)
#define log_trace_(format)
#define log_debug(format, ...)
#define log_debug_(format)
#define log_info(format, ...) 
#define log_info_(format)
#define log_warning(format, ...)
#define log_warning_(format)
#define log_error(format, ...)
#define log_error_(format)
#define log_critical(format, ...)
#define log_critical_(format)
#define log_success(format, ...)
#define log_success_(format)
#endif
// clang-format on
