#include "gesture_synchronizer/gesture_gui.h"
#include <vision_utils/to_lowercase.h>
#include "std_msgs/String.h"
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>

void KeyWidget::erase() {
  ROS_INFO("erase('%s')", to_string().c_str());
  gesture_synchronizer::clear_joint_at_given_time
      (_gui->_gesture, _joint_name, _keytime, _gui->_time_tolerance);
  _gui->refresh_list_view();
} // end erase()

////////////////////////////////////////////////////////////////////////////////

void KeyWidget::set_new_value() {
  ROS_INFO("set_new_value('%s':%s)", to_string().c_str(),
           _value_linedit->text().toLocal8Bit().constData());
  gesture_synchronizer::clear_joint_at_given_time
      (_gui->_gesture, _joint_name, _keytime, _gui->_time_tolerance);
  gesture_synchronizer::set_joint_at_given_time
      (_gui->_gesture, _joint_name,
       std::string(_value_linedit->text().toLocal8Bit().constData()), _keytime);
  _gui->move_to_current();
} // end erase()

////////////////////////////////////////////////////////////////////////////////

SubscriberWidget::SubscriberWidget
(GestureGui* gesture_gui, const gesture_synchronizer::JointName & joint_name)
  : WidgetModel(gesture_gui, joint_name){
  // set value widget
  _value_label = new QLabel(this);
  _value_label->setWordWrap(true);
  _layout->addWidget(_value_label);

  // set signals
  connect(_button, SIGNAL(clicked()), this, SLOT(add()));
  _button->setText("[+]");

  // add to gui
  _gui->ui.sub_jts_layout->addWidget(this);
} // end GeneralSubscribedWidget ctor

////////////////////////////////////////////////////////////////////////////////

void SubscriberWidget::add() {
  ROS_INFO("add('%s')", _joint_name.c_str());
  gesture_synchronizer::clear_joint_at_given_time
      (_gui->_gesture, _joint_name, _gui->current_time(), _gui->_time_tolerance);
  gesture_synchronizer::set_joint_at_given_time(_gui->_gesture,
                                      _joint_name,
                                      _joint_value,
                                      _gui->current_time());
  _gui->refresh_list_view();
} // end add()

///////////////////////////////////////////////////////////////////////////////

template<class T>
class CastableWidget : public SubscriberWidget {
public:
  CastableWidget(GestureGui* gesture_gui,
                 const gesture_synchronizer::JointName & joint_name) :
    SubscriberWidget(gesture_gui, joint_name)
  {
    ROS_INFO("CastableWidget ctor('%s')", _joint_name.c_str());
    _joint_state_sub = _nh_public.subscribe(joint_name, 1,
                                            &CastableWidget::joint_msg_callback, this);
  } // end CastableWidget ctor

protected:
  //! the callback when a message is received
  void joint_msg_callback(const typename T::ConstPtr & msg) {
    _joint_value = vision_utils::cast_to_string(msg->data);
    _value_label->setText(QString::fromStdString(_joint_value));
  } // end joint_msg_callback()

  ros::Subscriber _joint_state_sub;
}; // end class CastableWidget

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

GestureGui::GestureGui(QWidget *parent = 0) :
  QMainWindow(parent),
  _nh_private("~") {
  // setup ui - LEAVE THIS AT THE BEGINNING OF THE CONSTRUCTOR
  ui.setupUi(this);

  // find all topics of dynamixel
  std::string joints_concatenated;
  _nh_private.param<std::string>("joints", joints_concatenated, "");
  // split the concatenated joints
  std::vector<std::string> joints;
  vision_utils::StringSplit(joints_concatenated, ";", &joints);
  // error if no joints
  if (joints.empty()) {
    ROS_FATAL("GestureGui: you didn't specify any joints to listen to. "
              "Please set the command line  'arg _joints=joint1:string;joint2:string' "
              " for instance if your joints are 'joint1' and 'joint2, of types String.");
    ros::shutdown();
    exit(-1);
  }

  std::string subscribed_joints_configs =
      vision_utils::accessible_to_string(joints).c_str();
  ROS_INFO("subscribed_joints_configs:'%s'", subscribed_joints_configs.c_str());

  // subscribe to all joints states messages
  for (unsigned int joint_idx = 0; joint_idx < joints.size(); ++joint_idx) {
    // extract the params of the current config
    std::string current_config = joints[joint_idx];
    std::vector<std::string> current_config_params;
    vision_utils::StringSplit(current_config, ":", &current_config_params);
    if (current_config_params.size() != 2) {
      ROS_WARN("The current config %s does not contain two params separated by a ':'",
               current_config.c_str());
      continue;
    }
    std::string joint_name = current_config_params.at(0);
    std::string joint_type = current_config_params.at(1);
    vision_utils::to_lowercase(joint_type);
    if (joint_type == "string")
      _joints.push_back(new CastableWidget<std_msgs::String>(this, joint_name));
    else if (joint_type == "bool")
      _joints.push_back(new CastableWidget<std_msgs::Bool>(this, joint_name));
    else if (joint_type == "int8")
      _joints.push_back(new CastableWidget<std_msgs::Int8>(this, joint_name));
    else if (joint_type == "int16")
      _joints.push_back(new CastableWidget<std_msgs::Int16>(this, joint_name));
    else if (joint_type == "int32")
      _joints.push_back(new CastableWidget<std_msgs::Int32>(this, joint_name));
    else if (joint_type == "int64")
      _joints.push_back(new CastableWidget<std_msgs::Int64>(this, joint_name));
    else if (joint_type == "float32")
      _joints.push_back(new CastableWidget<std_msgs::Float32>(this, joint_name));
    else if (joint_type == "float64")
      _joints.push_back(new CastableWidget<std_msgs::Float64>(this, joint_name));
    else {
      ROS_WARN("The subscription type '%s' is unknown", joint_type.c_str());
    }
  } // end loop joint_idx

  // get time tolerance
  _time_tolerance = ui.time_slider->singleStep() / 10.f;
  ROS_DEBUG("time_tolerance:%g", _time_tolerance);

  // set signals
  connect(ui.actionOpen, SIGNAL(triggered()), this, SLOT(open_dialog()));
  connect(ui.actionSave, SIGNAL(triggered()), this, SLOT(save_dialog()));
  connect(ui.actionQuit, SIGNAL(triggered()), this, SLOT(close()));
  connect(ui.time_slider, SIGNAL(valueChanged(int)), this,
          SLOT(set_current_time_label()));
  connect(ui.time_slider, SIGNAL(valueChanged(int)), this,
          SLOT(refresh_list_view()));
  connect(ui.play_button, SIGNAL(clicked()), this, SLOT(play_gesture()));
  connect(ui.move_to_current_button, SIGNAL(clicked()), this, SLOT(move_to_current()));

  // init the slider label
  set_current_time_label();
  refresh_list_view();
} // end ctor

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv) {
  ros::init(argc, argv, "gesture_gui");
  ros::Time::init();
  ros::AsyncSpinner spinner(1);
  spinner.start();
  QApplication app(argc, argv);
  GestureGui gui;
  //gui.open_file(gesture_synchronizer::gesture_files_folder() +"/zero.xml");
  gui.show();
  app.exec();
  spinner.stop();
  return 0;
}
