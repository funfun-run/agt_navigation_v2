/*
 * @Author: chengyangkj chengyangkj@qq.com
 * @Date: 2023-09-28 14:56:04
 * @LastEditors: chengyangkj chengyangkj@qq.com
 * @LastEditTime: 2023-10-05 11:39:01
 * @FilePath: /ROS2_Qt5_Gui_App/src/app/main.cpp
 */
#ifndef SDL_MAIN_HANDLED
#define SDL_MAIN_HANDLED
#endif

#include <QApplication>
#include <QLabel>
#include <QMovie>
#include <QPixmap>
#include <QSplashScreen>
#include <QThread>
#include <QTimer>
#include <csignal>
#include <iostream>
#include "logger/logger.h"
#include "mainwindow.h"


static volatile std::sig_atomic_t g_exit_requested = 0;

void signalHandler(int signal) {
  if (signal == SIGINT || signal == SIGTERM) {
    g_exit_requested = 1;
  }
}

int main(int argc, char *argv[]) {
  QApplication a(argc, argv);
  std::signal(SIGINT, signalHandler);
  std::signal(SIGTERM, signalHandler);

  // Process termination requests on Qt's main thread.
  QTimer signal_timer;
  QObject::connect(&signal_timer, &QTimer::timeout, [&a]() {
    if (g_exit_requested) {
      a.quit();
    }
  });
  signal_timer.start(100);

  MainWindow main_window;
  main_window.show();
  LOG_INFO("ros_qt5_gui_app init!");
  return a.exec();
}
