/**
 * Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.
 *
 * This file is part of sunnypilot and is licensed under the MIT License.
 * See the LICENSE.md file in the root directory for more details.
 */

#pragma once

#include "selfdrive/ui/sunnypilot/qt/offroad/settings/settings.h"

class PlatformSelector : public ButtonControl {
  Q_OBJECT

public:
  PlatformSelector();

private:
  Params params;

  void refresh();
  void searchPlatforms(const QString &query);
  QMap<QString, QVariantMap> loadPlatformList();
};
