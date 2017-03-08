#ifndef GESTURE_GUI_H
#define GESTURE_GUI_H

// QT
#include <QApplication>
#include <QSlider>
#include <QLabel>
#include <QFileDialog>
#include <QPushButton>
#include <QHBoxLayout>
#include <QLineEdit>
// ROS
#ifndef Q_MOC_RUN // http://answers.ros.org/question/233786/parse-error-at-boost_join/
#include <ros/ros.h>
// vision_utils
#include <vision_utils/accessible_to_string.h>
#include <vision_utils/extract_filename_from_full_path.h>
// gesture_synchronizer
#include "gesture_synchronizer/gesture_definitions.h"
#include "gesture_synchronizer/gesture_synchronizer.h"
#include "src/ui_gesture_gui.h"
#endif //Q_MOC_RUN

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

class GestureGui; // forward declaration

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

class WidgetModel : public QWidget {
  Q_OBJECT
public:
  //! ctor
  WidgetModel(GestureGui* gesture_gui,
              const gesture_synchronizer::JointName & joint_name) :
    _gui(gesture_gui),
    _joint_name(joint_name),
    _joint_value("")
  {
    // create layout
    _layout = new QHBoxLayout(this);
    // create button
    _button = new QPushButton(this);
    _button->setMaximumSize(25, 25);
    _layout->addWidget(_button);
    // create label
    _joint_name_label = new QLabel(this);
    _joint_name_label->setText(QString::fromStdString(_joint_name));
    _layout->addWidget(_joint_name_label);
  } // end ctor

  //////////////////////////////////////////////////////////////////////////////

protected:
  // data
  GestureGui* _gui;
  gesture_synchronizer::JointName _joint_name;
  gesture_synchronizer::JointValue _joint_value;
  // Qt stuff
  QLabel* _joint_name_label;
  QPushButton* _button;
  QHBoxLayout* _layout;
}; // end class WidgetModel

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

class KeyWidget : public WidgetModel {
  Q_OBJECT
public:
  KeyWidget(GestureGui* gesture_gui,
            const gesture_synchronizer::JointName & joint_name,
            const gesture_synchronizer::JointValue & joint_value,
            const gesture_synchronizer::Time & time) :
    WidgetModel(gesture_gui, joint_name),
    _keytime(time)
  {
    _joint_value = joint_value;
    ROS_DEBUG("KeyWidget ctor('%s', %s)", to_string().c_str(),
              _joint_value.c_str());
    _button->setText("[-]");

    // set value widget
    _value_linedit = new QLineEdit(this);
    //_value_box->setFont(QFont( "Helvetica [Cronyx]", 9));
    //_value_spinbox->setSingleStep(0.1);
    //_value_spinbox->setRange(-M_PI_2, M_PI_2);
    _value_linedit->setText(QString::fromStdString(_joint_value));
    _layout->addWidget(_value_linedit);

    // set signals
    connect(_button, SIGNAL(clicked()), this, SLOT(erase()));
    connect(_value_linedit, SIGNAL(editingFinished()), this, SLOT(set_new_value()));
  } // end ctor

  ////////////////////////////////////////////////////////////////////////////

protected slots:
  void erase();

  void set_new_value();

  ////////////////////////////////////////////////////////////////////////////

protected:
  std::string to_string() {
    std::ostringstream ans;
    ans << "t:" << _keytime << ", " << _joint_name;
    return ans.str();
  } // end to_string()

  ////////////////////////////////////////////////////////////////////////////////

  QLineEdit* _value_linedit;
  gesture_synchronizer::Time _keytime;
}; // end class KeyWidget

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

class SubscriberWidget : public WidgetModel {
  Q_OBJECT
public:
  SubscriberWidget(GestureGui* gesture_gui,
                   const gesture_synchronizer::JointName & joint_name);  //optional parameter to allow
  //its childrens to override the
  //functionality in the constructor

protected slots:
  void add();

protected:
  ros::NodeHandle _nh_public;
  ros::Subscriber _state_sub;
  // Qt stuff
  QLabel* _value_label;
}; // end class GeneralSubscribedWidget

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

class GestureGui : public QMainWindow {
  Q_OBJECT
public:
  // ctor
  GestureGui(QWidget *parent /*= 0*/);

  //////////////////////////////////////////////////////////////////////////////

  void open_file(const std::string & filename) {
    ROS_DEBUG("Loading gesture from filename:%s", filename.c_str());
    gesture_synchronizer::load_from_xml_file(_gesture, filename);
    // set custom window title
    this->setWindowTitle("GestureGui - "
                         + QString::fromStdString
                         (vision_utils::extract_filename_from_full_path(filename)));
    // refresh the list
    refresh_list_view();
    // set status bar
    std::ostringstream status;
    status << "Succesfully loaded "
           << vision_utils::extract_filename_from_full_path(filename)
           << " (" << filename << ")";
    ui.statusbar->showMessage(QString::fromStdString(status.str()));
  } // end open_file();

  //////////////////////////////////////////////////////////////////////////////

  void save_file(const std::string & filename) {
    ROS_DEBUG("Saving gesture to file:%s", filename.c_str());
    gesture_synchronizer::save_to_xml_file_labeled(_gesture, filename);
    // set custom window title
    this->setWindowTitle("GestureGui - "
                         + QString::fromStdString
                         (vision_utils::extract_filename_from_full_path(filename)));
    // refresh the list
    refresh_list_view();
    // set status bar
    ui.statusbar->showMessage(QString("Succesfully saved to ")
                              + QString::fromStdString(filename));
  } // end save_file();

  //////////////////////////////////////////////////////////////////////////////

protected slots:
  void open_dialog() {
    QString filename_qstring = QFileDialog::getOpenFileName
        (this, QString("Open File"),
         QString::fromStdString(gesture_synchronizer::gesture_files_folder()),
         QString("Files (*.*)"));
    // do nothing if the user cancelled (pressed Esc)
    if (filename_qstring.size() == 0)
      return;
    open_file(filename_qstring.toStdString());
  }

  //////////////////////////////////////////////////////////////////////////////

  void save_dialog() {
    QString filename_qstring = QFileDialog::getSaveFileName
        (this, QString("Save file"),
         QString::fromStdString(gesture_synchronizer::gesture_files_folder()),
         QString("Files (*.*)"));
    // do nothing if the user cancelled (pressed Esc)
    if (filename_qstring.size() == 0)
      return;
    save_file(filename_qstring.toStdString());
  }

  //////////////////////////////////////////////////////////////////////////////

  void refresh_list_view() {
    ROS_DEBUG("refresh_list_view(), time:%g s", current_time());
    // clear the previous list
    for (int key_idx = _keys.size() - 1; key_idx >= 0; --key_idx) {
      ui.key_jts_layout->removeWidget(_keys[key_idx]);
      delete _keys[key_idx];
    } // end loop key_idx
    _keys.clear();

    // get all keys
    for (unsigned int key_idx = 0; key_idx < _gesture.keytimes.size(); ++key_idx) {
      // if time match
      if (fabs(_gesture.keytimes[key_idx] - current_time()) < _time_tolerance ) {
        // add the key
        KeyWidget* curr_label = new KeyWidget
            (this,
             _gesture.joint_names[key_idx].c_str(),
             _gesture.joint_values[key_idx],
             _gesture.keytimes[key_idx]);
        // store the reference
        _keys.push_back(curr_label);
        // add it to layout
        ui.key_jts_layout->addWidget(curr_label);
      } // end if time match
    } // end loop key_idx
  } // end refresh_list_view();

  //////////////////////////////////////////////////////////////////////////////

  inline void set_current_time_label() {
    ROS_DEBUG("set_current_time_label()");
    std::ostringstream label;
    label << "t:" << current_time() << " s";
    ui.time_slider_label->setText(QString::fromStdString(label.str()));
  } // end set_current_time_label();

  //////////////////////////////////////////////////////////////////////////////

  void play_gesture() {
    _gesture.header.stamp = ros::Time::now();
    ROS_DEBUG("play_gesture(id:%f)", _gesture.header.stamp.toSec());
    _gesture_pub.publish_gesture_and_wait(_gesture);
  } // end play_gesture()

  ////////////////////////////////////////////////////////////////////////////////

  void move_to_current() {
    ROS_DEBUG("move_to_current()");
    // create a new gesture with the position of all joints at current time
    gesture_msgs::KeyframeGesture temp_gesture;
    gesture_synchronizer::get_all_joints_at_given_time
        (_gesture, temp_gesture, current_time(), _time_tolerance);
    // set stamps to 0
    for (unsigned int i = 0; i < temp_gesture.keytimes.size(); ++i)
      temp_gesture.keytimes[i] = 0;
    temp_gesture.header.stamp = ros::Time::now();
    _gesture_pub.publish_gesture_and_wait(temp_gesture);
  } // end move_to_current()

protected:

  //These classes need to be "friends" to access the GestureGui* _gui inside
  //some of their methods
  friend class KeyWidget;
  friend class SubscriberWidget;
  friend class StringWidget;
  friend class ScreensSubscribedWidget;

  inline gesture_synchronizer::Time current_time() {
    return ui.time_slider->value() / 10.f;
  }

  //////////////////////////////////////////////////////////////////////////////

  ros::NodeHandle _nh_private, _nh_public;
  Ui::GestureGui ui;
  gesture_msgs::KeyframeGesture _gesture;
  gesture_synchronizer::Time _time_tolerance;
  //! the Qt widget list containing the widgets of keys at selected time
  std::vector<QWidget*> _keys;
  //! the Qt widget list containing the Qwidgets of joints at current time
  std::vector<QWidget*> _joints;
  //! the publisher for generated gestures
  GestureSynchronizer _gesture_pub;
}; // end class GestureGui

#endif // GESTURE_GUI_H
