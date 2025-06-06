/**
 * Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.
 *
 * This file is part of sunnypilot and is licensed under the MIT License.
 * See the LICENSE.md file in the root directory for more details.
 */

#include "selfdrive/ui/sunnypilot/qt/offroad/settings/lateral_panel.h"

#include "common/util.h"
#include "selfdrive/ui/sunnypilot/qt/widgets/controls.h"

LateralPanel::LateralPanel(SettingsWindowSP *parent) : QFrame(parent) {
  main_layout = new QStackedLayout(this);
  ListWidget *list = new ListWidget(this, false);

  sunnypilotScreen = new QWidget(this);
  QVBoxLayout* vlayout = new QVBoxLayout(sunnypilotScreen);
  vlayout->setContentsMargins(50, 20, 50, 20);

  // MADS
  madsToggle = new ParamControl(
    "Mads",
    tr("Modular Assistive Driving System (MADS)"),
    tr("Enable the beloved MADS feature. Disable toggle to revert back to stock sunnypilot engagement/disengagement."),
    "");
  madsToggle->setConfirmation(true, false);
  list->addItem(madsToggle);

  madsSettingsButton = new PushButtonSP(tr("Customize MADS"));
  madsSettingsButton->setObjectName("mads_btn");
  connect(madsSettingsButton, &QPushButton::clicked, [=]() {
    sunnypilotScroller->setLastScrollPosition();
    main_layout->setCurrentWidget(madsWidget);
  });
  QObject::connect(madsToggle, &ToggleControl::toggleFlipped, madsSettingsButton, &PushButtonSP::setEnabled);

  madsWidget = new MadsSettings(this);
  connect(madsWidget, &MadsSettings::backPress, [=]() {
    sunnypilotScroller->restoreScrollPosition();
    main_layout->setCurrentWidget(sunnypilotScreen);
  });
  list->addItem(madsSettingsButton);

  list->addItem(vertical_space());
  list->addItem(horizontal_line());
  list->addItem(vertical_space());

  // Lane Change Settings
  laneChangeSettingsButton = new PushButtonSP(tr("Customize Lane Change"));
  laneChangeSettingsButton->setObjectName("lane_change_btn");
  connect(laneChangeSettingsButton, &QPushButton::clicked, [=]() {
    sunnypilotScroller->setLastScrollPosition();
    main_layout->setCurrentWidget(laneChangeWidget);
  });

  laneChangeWidget = new LaneChangeSettings(this);
  connect(laneChangeWidget, &LaneChangeSettings::backPress, [=]() {
    sunnypilotScroller->restoreScrollPosition();
    main_layout->setCurrentWidget(sunnypilotScreen);
  });
  list->addItem(laneChangeSettingsButton);

  list->addItem(vertical_space(0));
  list->addItem(horizontal_line());

  // Neural Network Lateral Control
  nnlcToggle = new NeuralNetworkLateralControl();
  list->addItem(nnlcToggle);

  QObject::connect(nnlcToggle, &ParamControl::toggleFlipped, [=](bool state) {
    if (state) {
      nnlcToggle->showDescription();
    } else {
      nnlcToggle->hideDescription();
    }

    nnlcToggle->updateToggle();
  });

  toggleOffroadOnly = {
    madsToggle, nnlcToggle,
  };
  QObject::connect(uiState(), &UIState::offroadTransition, this, &LateralPanel::updateToggles);

  sunnypilotScroller = new ScrollViewSP(list, this);
  vlayout->addWidget(sunnypilotScroller);

  main_layout->addWidget(sunnypilotScreen);
  main_layout->addWidget(madsWidget);
  main_layout->addWidget(laneChangeWidget);

  setStyleSheet(R"(
    #back_btn {
      font-size: 50px;
      margin: 0px;
      padding: 15px;
      border-width: 0;
      border-radius: 30px;
      color: #dddddd;
      background-color: #393939;
    }
    #back_btn:pressed {
      background-color:  #4a4a4a;
    }
  )");

  main_layout->setCurrentWidget(sunnypilotScreen);
}

void LateralPanel::showEvent(QShowEvent *event) {
  nnlcToggle->updateToggle();
  updateToggles(offroad);
}

void LateralPanel::hideEvent(QHideEvent *event) {
  main_layout->setCurrentWidget(sunnypilotScreen);
}

void LateralPanel::updateToggles(bool _offroad) {
  for (auto *toggle : toggleOffroadOnly) {
    toggle->setEnabled(_offroad);
  }

  madsSettingsButton->setEnabled(madsToggle->isToggled());

  offroad = _offroad;
}
