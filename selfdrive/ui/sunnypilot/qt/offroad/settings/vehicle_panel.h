/**
 * Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.
 *
 * This file is part of sunnypilot and is licensed under the MIT License.
 * See the LICENSE.md file in the root directory for more details.
 */

#pragma once

#include "selfdrive/ui/sunnypilot/qt/offroad/settings/settings.h"

class VehiclePanel : public QFrame {
  Q_OBJECT

public:
  explicit VehiclePanel(QWidget *parent = nullptr);

private:
  QStackedLayout* main_layout = nullptr;
  QWidget* vehicleScreen = nullptr;
  ButtonControl* carSelector = nullptr;
};
