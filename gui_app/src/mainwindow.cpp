#include "gui_app/mainwindow.hpp"
#include <QPushButton>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QLabel>
#include <QStatusBar>
#include <QScrollArea>
#include <QPixmap>
#include <QFrame>
#include <qpushbutton.h>
#include <std_msgs/msg/detail/bool__struct.hpp>
#include <std_srvs/srv/detail/set_bool__struct.hpp>

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent) {
  // ── Global stylesheet ──────────────────────────────────────────────
  setStyleSheet(
    "QMainWindow { background-color: #1e2124; }"
    "QScrollArea { border: none; background-color: #1e2124; }"
    "QWidget { background-color: #1e2124; color: #c8cdd6; }"
    "QGroupBox {"
    "  font-size: 11px; font-weight: bold; letter-spacing: 1.5px;"
    "  color: #7a8494; border: 1px solid #32363e;"
    "  margin-top: 18px; padding: 12px 8px 8px 8px;"
    "  background-color: #23272b; border-radius: 2px; }"
    "QGroupBox::title {"
    "  subcontrol-origin: margin; left: 10px; top: 2px; padding: 0 4px; }"
    "QLabel { color: #8a909c; font-size: 12px; background: transparent; }"
    "QLineEdit {"
    "  background-color: #191b1f; border: 1px solid #32363e;"
    "  color: #c8cdd6; padding: 6px 10px; font-size: 12px; border-radius: 2px; }"
    "QLineEdit:focus { border-color: #5088b8; color: #dde2ea; }"
    "QStatusBar {"
    "  background-color: #191b1f; color: #7a8494; font-size: 11px;"
    "  border-top: 1px solid #32363e; }"
  );

  auto *scrollArea = new QScrollArea(this);
  auto *central = new QWidget();
  auto *mainLayout = new QVBoxLayout(central);
  mainLayout->setSpacing(0);
  mainLayout->setContentsMargins(0, 0, 0, 0);

  // ── Header bar ────────────────────────────────────────────────────
  auto *headerWidget = new QWidget(central);
  headerWidget->setFixedHeight(64);
  headerWidget->setStyleSheet(
    "QWidget { background-color: #191b1f; border-bottom: 1px solid #32363e; }"
    "QLabel { background: transparent; }"
  );
  auto *headerLayout = new QHBoxLayout(headerWidget);
  headerLayout->setContentsMargins(18, 0, 18, 0);
  headerLayout->setSpacing(14);

  QPixmap logo("/home/furkan/magician_ws/src/gui_app/png/images.png");
  if (!logo.isNull()) {
    auto *logoLabel = new QLabel(headerWidget);
    logoLabel->setPixmap(logo.scaledToHeight(50, Qt::SmoothTransformation));
    headerLayout->addWidget(logoLabel);
  }

  auto *titleLabel = new QLabel("PLC CONTROL SYSTEM", headerWidget);
  titleLabel->setStyleSheet(
    "QLabel { color: #d0d6e0; font-size: 14px; font-weight: bold; letter-spacing: 2px; }");
  headerLayout->addWidget(titleLabel);
  headerLayout->addStretch();

  auto *subtitleLabel = new QLabel("ROS2  ·  OPC-UA BRIDGE", headerWidget);
  subtitleLabel->setStyleSheet(
    "QLabel { color: #5a6270; font-size: 11px; letter-spacing: 1.5px; }");
  headerLayout->addWidget(subtitleLabel);
  mainLayout->addWidget(headerWidget);

  // ── Main content: two-column layout ──────────────────────────────
  auto *contentWidget = new QWidget(central);
  auto *contentLayout = new QHBoxLayout(contentWidget);
  contentLayout->setSpacing(8);
  contentLayout->setContentsMargins(10, 8, 10, 8);

  // ── Left panel ────────────────────────────────────────────────────
  auto *leftPanel = new QWidget(contentWidget);
  leftPanel->setStyleSheet("QWidget { background-color: #161719; }");
  auto *leftLayout = new QVBoxLayout(leftPanel);
  leftLayout->setSpacing(8);
  leftLayout->setContentsMargins(0, 0, 0, 0);

  // Speed
  auto *speedGroup = new QGroupBox("SPEED CONTROL", leftPanel);
  auto *speedLayout = new QHBoxLayout(speedGroup);
  speedLayout->setSpacing(5);
  auto *speedEdit = new QLineEdit(speedGroup);
  speedEdit->setPlaceholderText("value");
  const QString actionBtn =
    "QPushButton { background-color: #243348; color: #6a9ec0; border: 1px solid #2e4460;"
    "  padding: 6px 18px; font-size: 12px; font-weight: bold; letter-spacing: 1px; border-radius: 2px; }"
    "QPushButton:hover { background-color: #2a3d58; border-color: #5088b8; color: #90bede; }"
    "QPushButton:pressed { background-color: #1c2a3e; }"; 
  auto *btnSpeedSet = new QPushButton("SET", speedGroup);
  btnSpeedSet->setStyleSheet(actionBtn);
  speedLayout->addWidget(new QLabel("SPEED", speedGroup));
  speedLayout->addWidget(speedEdit, 1);
  speedLayout->addWidget(btnSpeedSet);
  leftLayout->addWidget(speedGroup);

  // Linear Axis
  auto *slidersGroup = new QGroupBox("LINEAR AXIS", leftPanel);
  auto *slidersLayout = new QGridLayout(slidersGroup);
  slidersLayout->setSpacing(5);
  slidersLayout->setColumnStretch(1, 1);
  auto *slider1Edit = new QLineEdit(slidersGroup);
  slider1Edit->setPlaceholderText("target pos");
  auto *btnslider1Set = new QPushButton("SET", slidersGroup);
  auto *btnslider1Go  = new QPushButton("MOVE", slidersGroup);
  auto *slider2Edit = new QLineEdit(slidersGroup);
  slider2Edit->setPlaceholderText("target pos");
  auto *btnslider2Set = new QPushButton("SET", slidersGroup);
  auto *btnslider2Go  = new QPushButton("MOVE", slidersGroup);
  btnslider1Set->setStyleSheet(actionBtn);
  btnslider1Go->setStyleSheet(actionBtn);
  btnslider2Set->setStyleSheet(actionBtn);
  btnslider2Go->setStyleSheet(actionBtn);
  slidersLayout->addWidget(new QLabel("AXIS 1", slidersGroup), 0, 0);
  slidersLayout->addWidget(slider1Edit, 0, 1);
  slidersLayout->addWidget(btnslider1Set, 0, 2);
  slidersLayout->addWidget(btnslider1Go, 0, 3);
  slidersLayout->addWidget(new QLabel("AXIS 2", slidersGroup), 1, 0);
  slidersLayout->addWidget(slider2Edit, 1, 1);
  slidersLayout->addWidget(btnslider2Set, 1, 2);
  slidersLayout->addWidget(btnslider2Go, 1, 3);
  leftLayout->addWidget(slidersGroup);

  // System Mode
  auto *cobotGroup = new QGroupBox("SYSTEM MODE", leftPanel);
  auto *cobotLayout = new QVBoxLayout(cobotGroup);
  cobotLayout->setSpacing(4);
  btnCobotModeToggle_    = createToggleButton("COBOT MODE");
  btnAutomaticModeToggle_ = createToggleButton("FULL AUTOMATIC");
  cobotLayout->addWidget(btnCobotModeToggle_);
  cobotLayout->addWidget(btnAutomaticModeToggle_);
  leftLayout->addWidget(cobotGroup);
  leftLayout->addStretch();
  contentLayout->addWidget(leftPanel, 4);

  // ── Right panel ───────────────────────────────────────────────────
  auto *rightPanel = new QWidget(contentWidget);
  rightPanel->setStyleSheet("QWidget { background-color: #161719; }");
  auto *rightLayout = new QVBoxLayout(rightPanel);
  rightLayout->setSpacing(8);
  rightLayout->setContentsMargins(0, 0, 0, 0);

  // Sensing Robot
  auto *sensingGroup = new QGroupBox("SENSING ROBOT", rightPanel);
  auto *sensingLayout = new QGridLayout(sensingGroup);
  sensingLayout->setSpacing(4);
  btnSensingCarbodyLocatedSt_    = createToggleButton("Carbody Located");
  btnSensingSafeTransferToggle_  = createToggleButton("Robot Home");
  btnSensingFinishedToggle_      = createToggleButton("Sensing Finished");
  btnSensingTouchFinishedToggle_ = createToggleButton("Touch Finished");
  btnSensingActiveToggle_        = createToggleButton("Sensing Active");
  btnSensingTouchActiveToggle_   = createToggleButton("Touch Active");
  btnSensingSlideCommandToggle_  = createToggleButton("Slide Command");
  btnSensingRunningToggle_       = createToggleButton("Running");
  sensingLayout->addWidget(btnSensingSafeTransferToggle_,    0, 0);
  sensingLayout->addWidget(btnSensingFinishedToggle_,        0, 1);
  sensingLayout->addWidget(btnSensingTouchFinishedToggle_,   1, 0);
  sensingLayout->addWidget(btnSensingActiveToggle_,          1, 1);
  sensingLayout->addWidget(btnSensingTouchActiveToggle_,     2, 0);
  sensingLayout->addWidget(btnSensingSlideCommandToggle_,    2, 1);
  sensingLayout->addWidget(btnSensingRunningToggle_,         3, 0);
  sensingLayout->addWidget(btnSensingCarbodyLocatedSt_,      3, 1);
  rightLayout->addWidget(sensingGroup);

  // Cleaning Robot
  auto *cleaningGroup = new QGroupBox("CLEANING ROBOT", rightPanel);
  auto *cleaningLayout = new QGridLayout(cleaningGroup);
  cleaningLayout->setSpacing(4);
  btnCleaningCarbodyLocatedSt_   = createToggleButton("Carbody Located");
  btnCleaningSafeTransferToggle_ = createToggleButton("Robot Home");
  btnCleaningFinishedToggle_     = createToggleButton("Cleaning Finished");
  btnCleaningActiveToggle_       = createToggleButton("Cleaning Active");
  btnCleaningSlideCommandToggle_ = createToggleButton("Slide Command");
  btnCleaningRunningToggle_      = createToggleButton("Running");
  cleaningLayout->addWidget(btnCleaningSafeTransferToggle_,  0, 0);
  cleaningLayout->addWidget(btnCleaningFinishedToggle_,      0, 1);
  cleaningLayout->addWidget(btnCleaningActiveToggle_,        1, 0);
  cleaningLayout->addWidget(btnCleaningSlideCommandToggle_,  1, 1);
  cleaningLayout->addWidget(btnCleaningRunningToggle_,       2, 0);
  cleaningLayout->addWidget(btnCleaningCarbodyLocatedSt_,    2, 1);
  rightLayout->addWidget(cleaningGroup);
  rightLayout->addStretch();
  contentLayout->addWidget(rightPanel, 6);

  mainLayout->addWidget(contentWidget, 1);

  scrollArea->setWidget(central);
  scrollArea->setWidgetResizable(true);
  setCentralWidget(scrollArea);
  setWindowTitle("Magician — PLC Control");
  resize(1080, 680);

  setup_ros();

  connect(btnSpeedSet, &QPushButton::clicked, [this, speedEdit]{
    bool ok=false; int v = speedEdit->text().toInt(&ok);
    if(ok) call_speed_set(v);
  });

  connect(btnslider1Set, &QPushButton::clicked, [this, slider1Edit]{
    bool ok=false; float v = slider1Edit->text().toFloat(&ok);
    if(ok) call_slider_set(cli_slider1_set_pos_,v);
  });
  
  connect(btnslider2Set, &QPushButton::clicked, [this, slider2Edit]{
      bool ok=false; float v = slider2Edit->text().toFloat(&ok);
      if(ok) call_slider_set(cli_slider2_set_pos_,v);
    });

  connect(btnslider1Go, &QPushButton::clicked, [this](bool checked){ 
    call_service(cli_slider1_go_pos_, checked);
  });

  connect(btnslider2Go, &QPushButton::clicked, [this](bool checked){ 
    call_service(cli_slider2_go_pos_, checked);
  });
  connect(btnCobotModeToggle_, &QPushButton::toggled, [this](bool checked){ 
    updateToggleButtonStyle(btnCobotModeToggle_, checked);
    call_service(cli_mod_cobot_, checked);
  });
 
  connect(btnAutomaticModeToggle_, &QPushButton::toggled, [this](bool checked){
    updateToggleButtonStyle(btnAutomaticModeToggle_,checked);
    call_service(cli_mod_automatic_, checked);
  });

  connect(btnSensingCarbodyLocatedSt_,&QPushButton::toggled,[this](bool checked){
    updateToggleButtonStyle(btnSensingCarbodyLocatedSt_, checked);
    call_service(cli_sensing_carbody_located_st_,checked);        
  });

  connect(btnSensingSafeTransferToggle_, &QPushButton::toggled, [this](bool checked){ 
    updateToggleButtonStyle(btnSensingSafeTransferToggle_, checked);
    call_service(cli_sensing_safetransfer_, checked);
  });
  
  connect(btnSensingFinishedToggle_, &QPushButton::toggled, [this](bool checked){ 
    updateToggleButtonStyle(btnSensingFinishedToggle_, checked);
    call_service(cli_sensing_finished_, checked);
  });
  
  connect(btnSensingTouchFinishedToggle_, &QPushButton::toggled, [this](bool checked){ 
    updateToggleButtonStyle(btnSensingTouchFinishedToggle_, checked);
    call_service(cli_sensing_touch_finished_, checked);
  });
  
  connect(btnSensingActiveToggle_, &QPushButton::toggled, [this](bool checked){ 
    updateToggleButtonStyle(btnSensingActiveToggle_, checked);
    call_service(cli_sensing_active_, checked);
  });
  
  connect(btnSensingTouchActiveToggle_, &QPushButton::toggled, [this](bool checked){ 
    updateToggleButtonStyle(btnSensingTouchActiveToggle_, checked);
    call_service(cli_sensing_touch_active_, checked);
  });
  
  connect(btnSensingSlideCommandToggle_, &QPushButton::toggled, [this](bool checked){ 
    updateToggleButtonStyle(btnSensingSlideCommandToggle_, checked);
    call_service(cli_sensing_slide_command_, checked);
  });
  
  connect(btnSensingRunningToggle_, &QPushButton::toggled, [this](bool checked){ 
    updateToggleButtonStyle(btnSensingRunningToggle_, checked);
    call_service(cli_sensing_running_, checked);
  });
 


  connect(btnCleaningCarbodyLocatedSt_, &QPushButton::toggled, [this](bool checked){
    updateToggleButtonStyle(btnCleaningCarbodyLocatedSt_, checked);
    call_service(cli_cleaning_carbody_located_st_, checked);
  });
  connect(btnCleaningSafeTransferToggle_, &QPushButton::toggled, [this](bool checked){ 
    updateToggleButtonStyle(btnCleaningSafeTransferToggle_, checked);
    call_service(cli_cleaning_safetransfer_, checked);
  });
  
  connect(btnCleaningFinishedToggle_, &QPushButton::toggled, [this](bool checked){ 
    updateToggleButtonStyle(btnCleaningFinishedToggle_, checked);
    call_service(cli_cleaning_finished_, checked);
  });
  
  connect(btnCleaningActiveToggle_, &QPushButton::toggled, [this](bool checked){ 
    updateToggleButtonStyle(btnCleaningActiveToggle_, checked);
    call_service(cli_cleaning_active_, checked);
  });
  
  connect(btnCleaningSlideCommandToggle_, &QPushButton::toggled, [this](bool checked){ 
    updateToggleButtonStyle(btnCleaningSlideCommandToggle_, checked);
    call_service(cli_cleaning_slide_command_, checked);
  });
  
  connect(btnCleaningRunningToggle_, &QPushButton::toggled, [this](bool checked){ 
    updateToggleButtonStyle(btnCleaningRunningToggle_, checked);
    call_service(cli_cleaning_running_, checked);
  });
}

MainWindow::~MainWindow() {}

QPushButton* MainWindow::createToggleButton(const QString& label) {
  auto *btn = new QPushButton(label + ":  OFF", this);
  btn->setCheckable(true);
  btn->setChecked(false);
  btn->setMinimumHeight(34);
  updateToggleButtonStyle(btn, false);
  return btn;
}

void MainWindow::updateToggleButtonStyle(QPushButton* btn, bool state) {
  QString baseLabel = btn->text().left(btn->text().lastIndexOf(":"));
  if (state) {
    btn->setText(baseLabel + ":  ON");
    btn->setStyleSheet(
      "QPushButton { background-color: #1a3028; color: #5ed68e; font-weight: bold;"
      "  border: 1px solid #2a4e3a; border-left: 3px solid #3ec06a;"
      "  font-size: 12px; padding: 7px 12px; border-radius: 2px; text-align: left; }"
      "QPushButton:hover { background-color: #1e3a30; color: #78e8a4; }"
    );
  } else {
    btn->setText(baseLabel + ":  OFF");
    btn->setStyleSheet(
      "QPushButton { background-color: #23272b; color: #7a8494; font-weight: bold;"
      "  border: 1px solid #32363e; border-left: 3px solid #3e4450;"
      "  font-size: 12px; padding: 7px 12px; border-radius: 2px; text-align: left; }"
      "QPushButton:hover { background-color: #292e34; color: #9aa2b0; }"
    );
  }
}
void MainWindow::setup_ros() {
  node_ = std::make_shared<rclcpp::Node>("gui_node");
  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(node_);
  
  cli_speed_ = node_->create_client<backend::srv::SetInt16>("/ros2_comm/speed_set");
  cli_mod_cobot_ = node_->create_client<std_srvs::srv::SetBool>("/ros2_comm/mod/cobot_mode_set");
  cli_mod_automatic_ = node_->create_client<std_srvs::srv::SetBool>("/ros2_comm/mod/full_automatic_mode_set");

  cli_sensing_carbody_located_st_ = node_->create_client<std_srvs::srv::SetBool>("/ros2_comm/sensing/carbody_located_set");
  cli_sensing_safetransfer_ = node_->create_client<std_srvs::srv::SetBool>("/ros2_comm/sensing/safetransfer_set");
  cli_sensing_finished_ = node_->create_client<std_srvs::srv::SetBool>("/ros2_comm/sensing/finished_set");
  cli_sensing_touch_finished_ = node_->create_client<std_srvs::srv::SetBool>("/ros2_comm/sensing/touch_finished_set");
  cli_sensing_active_ = node_->create_client<std_srvs::srv::SetBool>("/ros2_comm/sensing/active_set");
  cli_sensing_touch_active_ = node_->create_client<std_srvs::srv::SetBool>("/ros2_comm/sensing/touch_active_set");
  cli_sensing_slide_command_ = node_->create_client<std_srvs::srv::SetBool>("/ros2_comm/sensing/slide_command_set");
  cli_sensing_running_ = node_->create_client<std_srvs::srv::SetBool>("/ros2_comm/sensing/running");
 

  cli_cleaning_carbody_located_st_= node_->create_client<std_srvs::srv::SetBool>("/ros2_comm/cleaning/carbody_located_set");
  cli_cleaning_safetransfer_ = node_->create_client<std_srvs::srv::SetBool>("/ros2_comm/cleaning/safetransfer_set");
  cli_cleaning_finished_ = node_->create_client<std_srvs::srv::SetBool>("/ros2_comm/cleaning/cleaning_finished_set");
  cli_cleaning_active_ = node_->create_client<std_srvs::srv::SetBool>("/ros2_comm/cleaning/cleaning_active_set");
  cli_cleaning_slide_command_ = node_->create_client<std_srvs::srv::SetBool>("/ros2_comm/cleaning/slide_command_set");
  cli_cleaning_running_ = node_->create_client<std_srvs::srv::SetBool>("/ros2_comm/cleaning/running_set");

  cli_slider1_set_pos_ = node_->create_client<backend::srv::SetFloat32>("/ros2_comm/slider1/set_pos");
  cli_slider2_set_pos_ = node_->create_client<backend::srv::SetFloat32>("/ros2_comm/slider2/set_pos");
  cli_slider1_go_pos_  = node_->create_client<std_srvs::srv::SetBool>("/ros2_comm/slider1/go_pos");
  cli_slider2_go_pos_  = node_->create_client<std_srvs::srv::SetBool>("/ros2_comm/slider2/go_pos");



  sub_speed_ = node_->create_subscription<std_msgs::msg::Int16>(
    "/ros2_comm/speed", 10,
    [this](const std_msgs::msg::Int16::SharedPtr msg){
      QMetaObject::invokeMethod(this, [this, value=msg->data](){
        this->statusBar()->showMessage(QString("⚡ Current Speed: %1").arg(value), 3000);
        this->statusBar()->setStyleSheet("QStatusBar { background-color: #34495e; color: white; font-weight: bold; }");
      }, Qt::QueuedConnection);
    });
  
  sub_cobot_mode_ = node_->create_subscription<std_msgs::msg::Bool>(
    "/ros2_comm/mod/cobot", 10,
    [this](const std_msgs::msg::Bool::SharedPtr msg){
      QMetaObject::invokeMethod(this, [this, state=msg->data](){
        btnCobotModeToggle_->blockSignals(true);
        btnCobotModeToggle_->setChecked(state);
        updateToggleButtonStyle(btnCobotModeToggle_, state);
        btnCobotModeToggle_->blockSignals(false);
      }, Qt::QueuedConnection);
    });
  sub_automatic_mode_ = node_->create_subscription<std_msgs::msg::Bool>("/ros2_comm/mod/automatic",10,
    [this](const std_msgs::msg::Bool::SharedPtr msg){
      QMetaObject::invokeMethod(this,[this,state=msg->data](){
        btnAutomaticModeToggle_->blockSignals(true);
        btnAutomaticModeToggle_->setChecked(state);
        updateToggleButtonStyle(btnAutomaticModeToggle_,state);
        btnAutomaticModeToggle_->blockSignals(false);
      }, Qt::QueuedConnection);
    });
 
    
  sub_sensing_carbody_located_st_ = node_->create_subscription<std_msgs::msg::Bool>("/ros2_comm/sensing/carbody_located_status",10,
          [this](const std_msgs::msg::Bool::SharedPtr msg){
          QMetaObject::invokeMethod(this,[this,state=msg->data](){
           btnSensingCarbodyLocatedSt_->blockSignals(true);
           btnSensingCarbodyLocatedSt_->setChecked(state);
           updateToggleButtonStyle(btnSensingCarbodyLocatedSt_,state);
           btnSensingCarbodyLocatedSt_->blockSignals(false);
           },Qt::QueuedConnection);
          });

  sub_sensing_safetransfer_ = node_->create_subscription<std_msgs::msg::Bool>(
    "/ros2_comm/sensing/home_st", 10,
    [this](const std_msgs::msg::Bool::SharedPtr msg){
      QMetaObject::invokeMethod(this, [this, state=msg->data](){
        btnSensingSafeTransferToggle_->blockSignals(true);
        btnSensingSafeTransferToggle_->setChecked(state);
        updateToggleButtonStyle(btnSensingSafeTransferToggle_, state);
        btnSensingSafeTransferToggle_->blockSignals(false);
      }, Qt::QueuedConnection);
    });
  
  sub_sensing_finished_ = node_->create_subscription<std_msgs::msg::Bool>(
    "/ros2_comm/sensing/finished", 10,
    [this](const std_msgs::msg::Bool::SharedPtr msg){
      QMetaObject::invokeMethod(this, [this, state=msg->data](){
        btnSensingFinishedToggle_->blockSignals(true);
        btnSensingFinishedToggle_->setChecked(state);
        updateToggleButtonStyle(btnSensingFinishedToggle_, state);
        btnSensingFinishedToggle_->blockSignals(false);
      }, Qt::QueuedConnection);
    });
  
  sub_sensing_touch_finished_ = node_->create_subscription<std_msgs::msg::Bool>(
    "/ros2_comm/sensing/touch_finished", 10,
    [this](const std_msgs::msg::Bool::SharedPtr msg){
      QMetaObject::invokeMethod(this, [this, state=msg->data](){
        btnSensingTouchFinishedToggle_->blockSignals(true);
        btnSensingTouchFinishedToggle_->setChecked(state);
        updateToggleButtonStyle(btnSensingTouchFinishedToggle_, state);
        btnSensingTouchFinishedToggle_->blockSignals(false);
      }, Qt::QueuedConnection);
    });
  
  sub_sensing_active_ = node_->create_subscription<std_msgs::msg::Bool>(
    "/ros2_comm/sensing/sensing_active", 10,
    [this](const std_msgs::msg::Bool::SharedPtr msg){
      QMetaObject::invokeMethod(this, [this, state=msg->data](){
        btnSensingActiveToggle_->blockSignals(true);
        btnSensingActiveToggle_->setChecked(state);
        updateToggleButtonStyle(btnSensingActiveToggle_, state);
        btnSensingActiveToggle_->blockSignals(false);
      }, Qt::QueuedConnection);
    });
  
  sub_sensing_touch_active_ = node_->create_subscription<std_msgs::msg::Bool>(
    "/ros2_comm/sensing/touch_active", 10,
    [this](const std_msgs::msg::Bool::SharedPtr msg){
      QMetaObject::invokeMethod(this, [this, state=msg->data](){
        btnSensingTouchActiveToggle_->blockSignals(true);
        btnSensingTouchActiveToggle_->setChecked(state);
        updateToggleButtonStyle(btnSensingTouchActiveToggle_, state);
        btnSensingTouchActiveToggle_->blockSignals(false);
      }, Qt::QueuedConnection);
    });
  
  sub_sensing_slide_command_ = node_->create_subscription<std_msgs::msg::Bool>(
    "/ros2_comm/sensing/slide_command", 10,
    [this](const std_msgs::msg::Bool::SharedPtr msg){
      QMetaObject::invokeMethod(this, [this, state=msg->data](){
        btnSensingSlideCommandToggle_->blockSignals(true);
        btnSensingSlideCommandToggle_->setChecked(state);
        updateToggleButtonStyle(btnSensingSlideCommandToggle_, state);
        btnSensingSlideCommandToggle_->blockSignals(false);
      }, Qt::QueuedConnection);
    });
  
  sub_sensing_running_ = node_->create_subscription<std_msgs::msg::Bool>(
    "/ros2_comm/sensing/running", 10,
    [this](const std_msgs::msg::Bool::SharedPtr msg){
      QMetaObject::invokeMethod(this, [this, state=msg->data](){
        btnSensingRunningToggle_->blockSignals(true);
        btnSensingRunningToggle_->setChecked(state);
        updateToggleButtonStyle(btnSensingRunningToggle_, state);
        btnSensingRunningToggle_->blockSignals(false);
      }, Qt::QueuedConnection);
    });



    
  sub_sensing_carbody_located_st_ = node_->create_subscription<std_msgs::msg::Bool>("/ros2_comm/cleaning/carbody_located_status",10,
          [this](const std_msgs::msg::Bool::SharedPtr msg){
          QMetaObject::invokeMethod(this,[this,state=msg->data](){
           btnCleaningCarbodyLocatedSt_->blockSignals(true);
           btnCleaningCarbodyLocatedSt_->setChecked(state);
           updateToggleButtonStyle(btnCleaningCarbodyLocatedSt_,state);
           btnCleaningCarbodyLocatedSt_->blockSignals(false);
           },Qt::QueuedConnection);
          });

  sub_cleaning_safetransfer_ = node_->create_subscription<std_msgs::msg::Bool>(
    "/ros2_comm/cleaning/home_st", 10,
    [this](const std_msgs::msg::Bool::SharedPtr msg){
      QMetaObject::invokeMethod(this, [this, state=msg->data](){
        btnCleaningSafeTransferToggle_->blockSignals(true);
        btnCleaningSafeTransferToggle_->setChecked(state);
        updateToggleButtonStyle(btnCleaningSafeTransferToggle_, state);
        btnCleaningSafeTransferToggle_->blockSignals(false);
      }, Qt::QueuedConnection);
    });
  
  sub_cleaning_finished_ = node_->create_subscription<std_msgs::msg::Bool>(
    "/ros2_comm/cleaning/finished", 10,
    [this](const std_msgs::msg::Bool::SharedPtr msg){
      QMetaObject::invokeMethod(this, [this, state=msg->data](){
        btnCleaningFinishedToggle_->blockSignals(true);
        btnCleaningFinishedToggle_->setChecked(state);
        updateToggleButtonStyle(btnCleaningFinishedToggle_, state);
        btnCleaningFinishedToggle_->blockSignals(false);
      }, Qt::QueuedConnection);
    });
  
  sub_cleaning_active_ = node_->create_subscription<std_msgs::msg::Bool>(
    "/ros2_comm/cleaning/cleaning_active", 10,
    [this](const std_msgs::msg::Bool::SharedPtr msg){
      QMetaObject::invokeMethod(this, [this, state=msg->data](){
        btnCleaningActiveToggle_->blockSignals(true);
        btnCleaningActiveToggle_->setChecked(state);
        updateToggleButtonStyle(btnCleaningActiveToggle_, state);
        btnCleaningActiveToggle_->blockSignals(false);
      }, Qt::QueuedConnection);
    });
  
  sub_cleaning_slide_command_ = node_->create_subscription<std_msgs::msg::Bool>(
    "/ros2_comm/cleaning/slide_command", 10,
    [this](const std_msgs::msg::Bool::SharedPtr msg){
      QMetaObject::invokeMethod(this, [this, state=msg->data](){
        btnCleaningSlideCommandToggle_->blockSignals(true);
        btnCleaningSlideCommandToggle_->setChecked(state);
        updateToggleButtonStyle(btnCleaningSlideCommandToggle_, state);
        btnCleaningSlideCommandToggle_->blockSignals(false);
      }, Qt::QueuedConnection);
    });
  
  sub_cleaning_running_ = node_->create_subscription<std_msgs::msg::Bool>(
    "/ros2_comm/cleaning/running", 10,
    [this](const std_msgs::msg::Bool::SharedPtr msg){
      QMetaObject::invokeMethod(this, [this, state=msg->data](){
        btnCleaningRunningToggle_->blockSignals(true);
        btnCleaningRunningToggle_->setChecked(state);
        updateToggleButtonStyle(btnCleaningRunningToggle_, state);
        btnCleaningRunningToggle_->blockSignals(false);
      }, Qt::QueuedConnection);
    });

  ros_timer_ = new QTimer(this);
  connect(ros_timer_, &QTimer::timeout, [this](){
    executor_->spin_some();
  });
  ros_timer_->start(20); // 50Hz
}
void MainWindow::call_speed_set(int value) {
  if(!cli_speed_->wait_for_service(std::chrono::seconds(1))) {
    statusBar()->showMessage("Speed service not available!", 3000);
    return;
  }
  auto req = std::make_shared<backend::srv::SetInt16::Request>();
  req->data = value;
  cli_speed_->async_send_request(req);
  statusBar()->showMessage(QString(" Speed set to: %1").arg(value), 3000);
}


void MainWindow::call_slider_set(rclcpp::Client<backend::srv::SetFloat32>::SharedPtr client, float value){
if(!client->wait_for_service(std::chrono::seconds(1))){
  statusBar()->showMessage("slider service not available!",3000);
  return;
}
  auto req = std::make_shared<backend::srv::SetFloat32::Request>();
  req->data = value;
  client->async_send_request(req);
  statusBar()->showMessage(QString("Slider set to %1").arg(value),3000);
}

void MainWindow::call_service(rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client, bool value) {
  if(!client->wait_for_service(std::chrono::seconds(1))) {
    statusBar()->showMessage(" Service not available!", 2000);
    return;
  }
  auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
  req->data = value;
  client->async_send_request(req);
}
