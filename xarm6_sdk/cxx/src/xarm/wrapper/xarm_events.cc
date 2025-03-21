/**
 * Software License Agreement (MIT License)
 * 
 * Copyright (c) 2022, UFACTORY, Inc.
 * 
 * All rights reserved.
 * 
 * @author Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>
 */

#include "xarm/wrapper/xarm6_api.h"


template<typename CallableVector, typename FunctionVector, class... arguments>
void XArmAPI::_report_callback(CallableVector&& callbacks, FunctionVector&& functions, arguments&&... args) {
  for (size_t i = 0; i < callbacks.size(); i++) {
    if (callback_in_thread_) pool_.dispatch(callbacks[i], std::forward<arguments>(args)...);
    else pool_.commit(callbacks[i], std::forward<arguments>(args)...);
  }
  for (size_t i = 0; i < functions.size(); i++) {
    if (callback_in_thread_) pool_.dispatch(functions[i], std::forward<arguments>(args)...);
    else pool_.commit(functions[i], std::forward<arguments>(args)...);
  }
}

void XArmAPI::_report_data_callback(XArmReportData *report_data_ptr) {
  _report_callback(report_data_callbacks_, report_data_functions_, report_data_ptr);
}

void XArmAPI::_report_location_callback(void) {
  _report_callback(report_location_callbacks_, report_location_functions_, position, angles);
  // for (size_t i = 0; i < report_location_callbacks_.size(); i++) {
  // 	if (callback_in_thread_) pool_.dispatch(report_location_callbacks_[i], position, angles);
  // 	else pool_.commit(report_location_callbacks_[i], position, angles);
  // }
}

void XArmAPI::_report_connect_changed_callback(void) {
  bool connected = stream_tcp_ == NULL ? false : stream_tcp_->is_ok() == 0;
  bool reported = stream_tcp_report_ == NULL ? false : stream_tcp_report_->is_ok() == 0;
  _report_callback(connect_changed_callbacks_, connect_changed_functions_, connected, reported);
  // for (size_t i = 0; i < connect_changed_callbacks_.size(); i++) {
  // 	if (callback_in_thread_) pool_.dispatch(connect_changed_callbacks_[i], connected, reported);
  // 	else pool_.commit(connect_changed_callbacks_[i], connected, reported);
  // }
}

void XArmAPI::_report_state_changed_callback(void) {
  if (ignore_state_) return;
  _report_callback(state_changed_callbacks_, state_changed_functions_, state);
  // for (size_t i = 0; i < state_changed_callbacks_.size(); i++) {
  // 	if (callback_in_thread_) pool_.dispatch(state_changed_callbacks_[i], state);
  // 	else pool_.commit(state_changed_callbacks_[i], state);
  // }
}

void XArmAPI::_report_mode_changed_callback(void) {
  _report_callback(mode_changed_callbacks_, mode_changed_functions_, mode);
  // for (size_t i = 0; i < mode_changed_callbacks_.size(); i++) {
  // 	if (callback_in_thread_) pool_.dispatch(mode_changed_callbacks_[i], mode);
  // 	else pool_.commit(mode_changed_callbacks_[i], mode);
  // }
}

void XArmAPI::_report_mtable_mtbrake_changed_callback(void) {
  _report_callback(mtable_mtbrake_changed_callbacks_, mtable_mtbrake_changed_functions_, mt_able_, mt_brake_);
  // for (size_t i = 0; i < mtable_mtbrake_changed_callbacks_.size(); i++) {
  // 	if (callback_in_thread_) pool_.dispatch(mtable_mtbrake_changed_callbacks_[i], mt_able_, mt_brake_);
  // 	else pool_.commit(mtable_mtbrake_changed_callbacks_[i], mt_able_, mt_brake_);
  // }
}

void XArmAPI::_report_error_warn_changed_callback(void) {
  if (ignore_error_) return;
  _report_callback(error_warn_changed_callbacks_, error_warn_changed_functions_, error_code, warn_code);
  // for (size_t i = 0; i < error_warn_changed_callbacks_.size(); i++) {
  // 	if (callback_in_thread_) pool_.dispatch(error_warn_changed_callbacks_[i], error_code, warn_code);
  // 	else pool_.commit(error_warn_changed_callbacks_[i], error_code, warn_code);
  // }
}

void XArmAPI::_report_cmdnum_changed_callback(void) {
  _report_callback(cmdnum_changed_callbacks_, cmdnum_changed_functions_, cmd_num);
  // for (size_t i = 0; i < cmdnum_changed_callbacks_.size(); i++) {
  // 	if (callback_in_thread_) pool_.dispatch(cmdnum_changed_callbacks_[i], cmd_num);
  // 	else pool_.commit(cmdnum_changed_callbacks_[i], cmd_num);
  // }
}

void XArmAPI::_report_temperature_changed_callback(void) {
  _report_callback(temperature_changed_callbacks_, temperature_changed_functions_, temperatures);
  // for (size_t i = 0; i < temperature_changed_callbacks_.size(); i++) {
  // 	if (callback_in_thread_) pool_.dispatch(temperature_changed_callbacks_[i], temperatures);
  // 	else pool_.commit(temperature_changed_callbacks_[i], temperatures);
  // }
}

void XArmAPI::_report_count_changed_callback(void) {
  _report_callback(count_changed_callbacks_, count_changed_functions_, count);
  // for (size_t i = 0; i < count_changed_callbacks_.size(); i++) {
  // 	if (callback_in_thread_) pool_.dispatch(count_changed_callbacks_[i], count);
  // 	else pool_.commit(count_changed_callbacks_[i], count);
  // }
}

void XArmAPI::_report_iden_progress_changed_callback(void) {
  _report_callback(iden_progress_changed_callbacks_, iden_progress_changed_functions_, iden_progress);
  // for (size_t i = 0; i < iden_progress_changed_callbacks_.size(); i++) {
  // 	if (callback_in_thread_) pool_.dispatch(iden_progress_changed_callbacks_[i], iden_progress);
  // 	else pool_.commit(iden_progress_changed_callbacks_[i], iden_progress);
  // }
}

void XArmAPI::_feedback_callback(unsigned char *feedback_data)
{
  unsigned short trans_id = bin8_to_16(&feedback_data[0]);
  int feedback_type = -1;
  if (fb_transid_type_map_.count(trans_id)) {
    feedback_type = fb_transid_type_map_[trans_id];
    fb_transid_type_map_.erase(trans_id);
  }
  if (feedback_type != -1) {
    fb_transid_result_map_[trans_id] = feedback_data[12];
  }
  if ((feedback_type & feedback_data[8]) == 0) return;
  _report_callback(feedback_callbacks_, feedback_functions_, feedback_data);
  // for (size_t i = 0; i < feedback_callbacks_.size(); i++) {
  // 	if (callback_in_thread_) pool_.dispatch(feedback_callbacks_[i], feedback_data);
  // 	else pool_.commit(feedback_callbacks_[i], feedback_data);
  // }
}

template<typename CallableVector, typename Callable>
int XArmAPI::_register_event_callback(CallableVector&& callbacks, Callable&& callback) {
  for (size_t i = 0; i < callbacks.size(); i++) {
    if (callbacks[i] == callback) return 1;
  }
  callbacks.push_back(callback);
  return 0;
}

template<typename CallableVector, typename Callable>
int XArmAPI::_release_event_callback(CallableVector&& callbacks, Callable&& callback) {
  if (callback == NULL) {
    callbacks.clear();
    return 0;
  }
  for (size_t i = 0; i < callbacks.size(); i++) {
    if (callbacks[i] == callback) {
      callbacks.erase(callbacks.begin() + i);
      return 0;
    }
  }
  return -1;
}

template<typename FunctionVector, typename Function>
int XArmAPI::_register_event_function(FunctionVector&& functions, Function&& function) {
  functions.push_back(function);
  return 0;
}

template<typename CallableVector, typename FunctionVector>
int XArmAPI::_clear_event_callback(CallableVector&& callbacks, FunctionVector&& functions, bool clear_all) {
  if (clear_all)
    callbacks.clear();
  functions.clear();
  return 0;
}


int XArmAPI::register_report_data_callback(void(*callback)(XArmReportData *report_data_ptr)) {
  return _register_event_callback(report_data_callbacks_, callback);
}

int XArmAPI::register_report_data_callback(std::function<void (XArmReportData *)> callback) {
  return _register_event_function(report_data_functions_, callback);
}

int XArmAPI::register_report_location_callback(void(*callback)(const fp32 *pose, const fp32 *angles)) {
  return _register_event_callback(report_location_callbacks_, callback);
}
int XArmAPI::register_report_location_callback(std::function<void (const fp32*, const fp32*)> callback) {
  return _register_event_function(report_location_functions_, callback);
}

int XArmAPI::register_connect_changed_callback(void(*callback)(bool connected, bool reported)) {
  return _register_event_callback(connect_changed_callbacks_, callback);
}
int XArmAPI::register_connect_changed_callback(std::function<void (bool, bool)> callback) {
  return _register_event_function(connect_changed_functions_, callback);
}

int XArmAPI::register_state_changed_callback(void(*callback)(int state)) {
  return _register_event_callback(state_changed_callbacks_, callback);
}
int XArmAPI::register_state_changed_callback(std::function<void (int)> callback) {
  return _register_event_function(state_changed_functions_, callback);
}

int XArmAPI::register_mode_changed_callback(void(*callback)(int mode)) {
  return _register_event_callback(mode_changed_callbacks_, callback);
}
int XArmAPI::register_mode_changed_callback(std::function<void (int)> callback) {
  return _register_event_function(mode_changed_functions_, callback);
}

int XArmAPI::register_mtable_mtbrake_changed_callback(void(*callback)(int mtable, int mtbrake)) {
  return _register_event_callback(mtable_mtbrake_changed_callbacks_, callback);
}
int XArmAPI::register_mtable_mtbrake_changed_callback(std::function<void (int, int)> callback) {
  return _register_event_function(mtable_mtbrake_changed_functions_, callback);
}

int XArmAPI::register_error_warn_changed_callback(void(*callback)(int err_code, int warn_code)) {
  return _register_event_callback(error_warn_changed_callbacks_, callback);
}
int XArmAPI::register_error_warn_changed_callback(std::function<void (int, int)> callback) {
  return _register_event_function(error_warn_changed_functions_, callback);
}

int XArmAPI::register_cmdnum_changed_callback(void(*callback)(int cmdnum)) {
  return _register_event_callback(cmdnum_changed_callbacks_, callback);
}
int XArmAPI::register_cmdnum_changed_callback(std::function<void (int)> callback) {
  return _register_event_function(cmdnum_changed_functions_, callback);
}

int XArmAPI::register_temperature_changed_callback(void(*callback)(const fp32 *temps)) {
  return _register_event_callback(temperature_changed_callbacks_, callback);
}
int XArmAPI::register_temperature_changed_callback(std::function<void (const fp32*)> callback) {
  return _register_event_function(temperature_changed_functions_, callback);
}

int XArmAPI::register_count_changed_callback(void(*callback)(int count)) {
  return _register_event_callback(count_changed_callbacks_, callback);
}
int XArmAPI::register_count_changed_callback(std::function<void (int)> callback) {
  return _register_event_function(count_changed_functions_, callback);
}

int XArmAPI::register_iden_progress_changed_callback(void(*callback)(int progress)) {
  return _register_event_callback(iden_progress_changed_callbacks_, callback);
}
int XArmAPI::register_iden_progress_changed_callback(std::function<void (int)> callback) {
  return _register_event_function(iden_progress_changed_functions_, callback);
}

int XArmAPI::register_feedback_callback(void(*callback)(unsigned char *feedback_data)) {
  return _register_event_callback(feedback_callbacks_, callback);
}
int XArmAPI::register_feedback_callback(std::function<void (unsigned char *feedback_data)> callback) {
  return _register_event_function(feedback_functions_, callback);
}

int XArmAPI::release_report_data_callback(void(*callback)(XArmReportData *report_data_ptr)) {
  return _release_event_callback(report_data_callbacks_, callback);
}

int XArmAPI::release_report_data_callback(bool clear_all) {
  return _clear_event_callback(report_data_callbacks_, report_data_functions_, clear_all);
}

int XArmAPI::release_report_location_callback(void(*callback)(const fp32 *pose, const fp32 *angles)) {
  return _release_event_callback(report_location_callbacks_, callback);
}
int XArmAPI::release_report_location_callback(bool clear_all) {
  return _clear_event_callback(report_location_callbacks_, report_location_functions_, clear_all);
}

int XArmAPI::release_connect_changed_callback(void(*callback)(bool connected, bool reported)) {
  return _release_event_callback(connect_changed_callbacks_, callback);
}
int XArmAPI::release_connect_changed_callback(bool clear_all) {
  return _clear_event_callback(connect_changed_callbacks_, connect_changed_functions_, clear_all);
}

int XArmAPI::release_state_changed_callback(void(*callback)(int state)) {
  return _release_event_callback(state_changed_callbacks_, callback);
}
int XArmAPI::release_state_changed_callback(bool clear_all) {
  return _clear_event_callback(state_changed_callbacks_, state_changed_functions_, clear_all);
}

int XArmAPI::release_mode_changed_callback(void(*callback)(int mode)) {
  return _release_event_callback(mode_changed_callbacks_, callback);
}
int XArmAPI::release_mode_changed_callback(bool clear_all) {
  return _clear_event_callback(mode_changed_callbacks_, mode_changed_functions_, clear_all);
}

int XArmAPI::release_mtable_mtbrake_changed_callback(void(*callback)(int mtable, int mtbrake)) {
  return _release_event_callback(mtable_mtbrake_changed_callbacks_, callback);
}
int XArmAPI::release_mtable_mtbrake_changed_callback(bool clear_all) {
  return _clear_event_callback(mtable_mtbrake_changed_callbacks_, mtable_mtbrake_changed_functions_, clear_all);
}

int XArmAPI::release_error_warn_changed_callback(void(*callback)(int err_code, int warn_code)) {
  return _release_event_callback(error_warn_changed_callbacks_, callback);
}
int XArmAPI::release_error_warn_changed_callback(bool clear_all) {
  return _clear_event_callback(error_warn_changed_callbacks_, error_warn_changed_functions_, clear_all);
}

int XArmAPI::release_cmdnum_changed_callback(void(*callback)(int cmdnum)) {
  return _release_event_callback(cmdnum_changed_callbacks_, callback);
}
int XArmAPI::release_cmdnum_changed_callback(bool clear_all) {
  return _clear_event_callback(cmdnum_changed_callbacks_, cmdnum_changed_functions_, clear_all);
}

int XArmAPI::release_temperature_changed_callback(void(*callback)(const fp32 *temps)) {
  return _release_event_callback(temperature_changed_callbacks_, callback);
}
int XArmAPI::release_temperature_changed_callback(bool clear_all) {
  return _clear_event_callback(temperature_changed_callbacks_, temperature_changed_functions_, clear_all);
}

int XArmAPI::release_count_changed_callback(void(*callback)(int count)) {
  return _release_event_callback(count_changed_callbacks_, callback);
}
int XArmAPI::release_count_changed_callback(bool clear_all) {
  return _clear_event_callback(count_changed_callbacks_, count_changed_functions_, clear_all);
}

int XArmAPI::release_iden_progress_changed_callback(void(*callback)(int progress)) {
  return _release_event_callback(iden_progress_changed_callbacks_, callback);
}
int XArmAPI::release_iden_progress_changed_callback(bool clear_all) {
  return _clear_event_callback(iden_progress_changed_callbacks_, iden_progress_changed_functions_, clear_all);
}

int XArmAPI::release_feedback_callback(void(*callback)(unsigned char *feedback_data)) {
  return _release_event_callback(feedback_callbacks_, callback);
}
int XArmAPI::release_feedback_callback(bool clear_all) {
  return _clear_event_callback(feedback_callbacks_, feedback_functions_, clear_all);
}
