#ifndef GESTURE_IO_H
#define GESTURE_IO_H

/*!
  This file contains useful classes for input
  on gesture_msgs::KeyframeGesture.

  For instance, you can export them to an XML format
  or import them from it.
  */

// generated message for gesture
#include <gesture_msgs/KeyframeGesture.h>
// ROS
#include <ros/ros.h>
// gesture_synchronizer
#include "gesture_definitions.h"
#include "vision_utils/XmlDocument.h"
#include <vision_utils/sort_utils.h>
#include <vision_utils/replace_find_tags.h>

namespace gesture_synchronizer {
typedef double Time;
typedef std::string JointName;
typedef std::string JointValue;

typedef gesture_msgs::KeyframeGesture KeyframeGesture;

////////////////////////////////////////////////////////////////////////////////

struct KeyframeGesture_Data{
  Time keytime;
  std::string joint_value;
  std::string joint_name;
};

////////////////////////////////////////////////////////////////////////////////
// Functions to sort gestures according to the keytimes
////////////////////////////////////////////////////////////////////////////////
inline bool compareKeyframeGesture(KeyframeGesture_Data const & a, KeyframeGesture_Data const & b){
  return a.keytime < b.keytime
      || (a.keytime == b.keytime && a.joint_name < b.joint_name);
}

////////////////////////////////////////////////////////////////////////////////
inline void sort_by_time(KeyframeGesture & gesture){
  // fill in the structur
  std::vector<KeyframeGesture_Data> vector_data;
  for(unsigned int i=0; i< gesture.keytimes.size(); i++){
    KeyframeGesture_Data data;
    data.keytime = gesture.keytimes[i];
    data.joint_value = gesture.joint_values[i];
    data.joint_name = gesture.joint_names[i];
    vector_data.push_back(data);
  }

  //sort structure
  std::sort(vector_data.begin(), vector_data.end(), compareKeyframeGesture);

  // copy back out into the vectors
  for(unsigned int i=0; i<vector_data.size(); i++){
    gesture.keytimes[i] = vector_data[i].keytime;
    gesture.joint_values[i]= vector_data[i].joint_value;
    gesture.joint_names[i]= vector_data[i].joint_name;
  }
}

////////////////////////////////////////////////////////////////////////////////

inline void print_keys(const KeyframeGesture & gesture) {
  for (unsigned key_idx = 0; key_idx < gesture.keytimes.size(); ++key_idx) {
    std::cout << "[" << gesture.keytimes[key_idx] << "]: \t"
              << gesture.joint_names[key_idx] << " : "
              << gesture.joint_values[key_idx]
                 << std::endl;
  } // end loop key_idx
} // end print_keys();

////////////////////////////////////////////////////////////////////////////////
/*! returns true if the joint has been inserted at the correct time
    or false if there is an error such as a negative keytime
*/
inline bool set_joint_at_given_time(KeyframeGesture & gesture,
                                    const JointName  & joint_name,
                                    const JointValue & joint_value,
                                    const Time & time_sec) {

  // check correctness of the time
  if (time_sec < 0)
    return false;

  // find appropriate position to insert it
  unsigned int key_idx = 0;
  for (key_idx = 0; key_idx < gesture.keytimes.size(); ++key_idx) {
    std::string curr_name = gesture.joint_names[key_idx];
    if ((gesture.keytimes[key_idx] == time_sec
         && curr_name > joint_name)
        || gesture.keytimes[key_idx] > time_sec)
      break;
  } // end loop key_idx

  // insert the gesture at given time
  gesture.keytimes.insert(gesture.keytimes.begin() + key_idx, time_sec);
  gesture.joint_names.insert(gesture.joint_names.begin() + key_idx, joint_name);
  gesture.joint_values.insert(gesture.joint_values.begin() + key_idx, joint_value);
  ROS_INFO("Added jointName: %s on key_idx: %d  size = %li",
           joint_name.c_str(), key_idx, gesture.keytimes.size());
  return true;
}

//! a templated version
template<class _T>
inline bool set_joint_at_given_time_(KeyframeGesture & gesture,
                                    const JointName  & joint_name,
                                    const _T & joint_value,
                                    const Time & time_sec) {
  return set_joint_at_given_time(gesture, joint_name,
                                 vision_utils::cast_to_string<_T>(joint_value),
                                 time_sec);
}

////////////////////////////////////////////////////////////////////////////////
/*!
  Get all the keys of a gesture at a given time
 \param gesture_in
 \param gesture_out
    Where the keys are saved. They are associated with a 0 time.
 \param time_sec
    The time wanted in gesture_in.
 \param time_tolerance
*/
inline void get_all_joints_at_given_time(const KeyframeGesture & gesture_in,
                                         KeyframeGesture & gesture_out,
                                         const Time & time_sec,
                                         const Time & time_tolerance) {
  gesture_out.keytimes.clear();
  gesture_out.joint_names.clear();
  gesture_out.joint_values.clear();
  for (unsigned int key_idx = 0; key_idx < gesture_in.keytimes.size(); ++key_idx) {
    // if time match
    if (fabs(gesture_in.keytimes[key_idx] - time_sec) <= time_tolerance) {
      // delete the key
      gesture_out.keytimes.push_back(gesture_in.keytimes[key_idx]);
      gesture_out.joint_names.push_back(gesture_in.joint_names[key_idx]);
      gesture_out.joint_values.push_back(gesture_in.joint_values[key_idx]);
    } // end if time match
  } // end loop key_idx
} // end get_all_joints_at_given_time();

////////////////////////////////////////////////////////////////////////////////

//! if joint_name = "", there is no joint_name test (faster)
inline void clear_joint_at_given_time(KeyframeGesture & gesture,
                                      const JointName & joint_name,
                                      const Time & time_sec,
                                      const Time & time_tolerance) {
  for (unsigned int key_idx = 0; key_idx < gesture.keytimes.size(); ++key_idx) {
    // if time match
    if (fabs(gesture.keytimes.at(key_idx) - time_sec) <= time_tolerance
        && (joint_name.empty() ||
            gesture.joint_names[key_idx].find(joint_name) != std::string::npos)) {
      // delete the key
      gesture.keytimes.erase(gesture.keytimes.begin() + key_idx);
      gesture.joint_names.erase(gesture.joint_names.begin() + key_idx);
      gesture.joint_values.erase(gesture.joint_values.begin() + key_idx);
      // rewind iterator
      --key_idx;
    } // end if time match
  } // end loop key_idx
} // end clear_joint_at_given_time();

////////////////////////////////////////////////////////////////////////////////

inline void clear_all_joints_at_given_time(KeyframeGesture & gesture,
                                           const Time & time_sec,
                                           const Time & time_tolerance) {
  clear_joint_at_given_time(gesture, "", time_sec, time_tolerance);
} // end clear_all_joints_at_given_time();

////////////////////////////////////////////////////////////////////////////////

inline void clear_gesture(KeyframeGesture & gesture) {
  gesture.keytimes.clear();
  gesture.joint_names.clear();
  gesture.joint_values.clear();
} // end clear_all_joints_at_given_time();

////////////////////////////////////////////////////////////////////////////////

inline void get_gesture_min_max_times(const KeyframeGesture & gesture,
                                      Time & gesture_min_time,
                                      Time & gesture_max_time) {
  // add all the keys corresponding to the wanted joint
  gesture_min_time = 0; // a gesture always starts at 0
  gesture_max_time = std::numeric_limits<Time>::min();

  for (unsigned int key_idx = 0; key_idx < gesture.keytimes.size(); ++key_idx) {
    // update min and max time for gesture
    gesture_min_time = std::min(gesture_min_time, gesture.keytimes[key_idx]);
    gesture_max_time = std::max(gesture_max_time, gesture.keytimes[key_idx]);
  } // end loop key_idx
} // end get_gesture_min_max_times();

////////////////////////////////////////////////////////////////////////////////
// This function is used in joint_player in order to extract all keys
// related to a specific joint_name (i.e. etts)
////////////////////////////////////////////////////////////////////////////////
//! extract all keys for a given joint
inline bool extract_all_joint_keys(const KeyframeGesture & gesture,
                                   const JointName & joint_name,
                                   std::vector<Time> & keytimes,
                                   std::vector<JointValue> & joint_values) {
  keytimes.clear();
  joint_values.clear();

  // add all the keys corresponding to the wanted joint
  for (unsigned int key_idx = 0; key_idx < gesture.keytimes.size(); ++key_idx) {
    // check if the current key is the joint of interest
    if (gesture.joint_names[key_idx] == joint_name) {
      keytimes.    push_back( gesture.keytimes[key_idx] );
      joint_values.push_back( gesture.joint_values[key_idx] );
    }
  } // end loop key_idx
  return true;
} // end extract_all_joint_keys()

////////////////////////////////////////////////////////////////////////////////

inline bool load_gestures(vision_utils::XmlDocument & doc, KeyframeGesture & gesture, const double startTime) {

  std::vector<vision_utils::XmlDocument::Node*> gesture_nodes;
  doc.get_all_nodes_at_direction(doc.root(), "gestures.gesture", gesture_nodes);
  //ROS_INFO("load_gestures:  %d",gesture_nodes.size());
  for (unsigned int key_idx = 0; key_idx < gesture_nodes.size(); ++key_idx) {
    // read attributes
    Time keytime =
        doc.get_node_attribute<Time>(gesture_nodes[key_idx], "keytime", 0);
    keytime+=startTime;

    JointName joint_name =
        doc.get_node_attribute<JointName>(gesture_nodes[key_idx], "joint_name", "");
    JointValue joint_value =
        doc.get_node_attribute<JointValue>(gesture_nodes[key_idx], "joint_value", "");

    //ROS_ERROR("load_gestures:  key_idx: %d ",key_idx);
    // push them into the gesture making sure they are ordered according to the
    // keytime
    //    set_joint_at_given_time(gesture, joint_name, joint_value, keytime);
    gesture.keytimes.push_back(keytime);
    gesture.joint_names.push_back(joint_name);
    gesture.joint_values.push_back(joint_value);
  } // end loop key_idx

  return true;
}

////////////////////////////////////////////////////////////////////////////////

// forward declaration
bool load_from_xml_file2(KeyframeGesture & gesture,
                         const std::string & xml_filename, const double startTime);

inline bool load_inner_xml_file(vision_utils::XmlDocument & doc, KeyframeGesture & gesture) {
  std::vector<vision_utils::XmlDocument::Node*> files_nodes;
  doc.get_all_nodes_at_direction(doc.root(), "gestures.include", files_nodes);
  //ROS_ERROR("load_inner_xml_file:  %d",files_nodes.size());
  for (unsigned int key_idx = 0; key_idx < files_nodes.size(); ++key_idx) {
    // read attributes ==>
    //<include keytime=[float] file=[string] repetitions=[integer] gap=[float]>

    Time keytime =
        doc.get_node_attribute<Time>(files_nodes[key_idx], "keytime", 0);
    std::string fileName =
        doc.get_node_attribute<std::string>(files_nodes[key_idx], "file", "");
    int repetitions=
        doc.get_node_attribute<int>(files_nodes[key_idx], "repetitions", 1);
    double gap=
        doc.get_node_attribute<double>(files_nodes[key_idx], "gap", 0);

    //read file
    if(!load_from_xml_file2(gesture, fileName, keytime))
      return false;
    //read the other reps
    for (int rep = 1; rep < repetitions; ++rep) {
      if(!load_from_xml_file2(gesture, fileName, keytime+(gap*rep)))
        return false;
    }

  }
  //ROS_ERROR("load_inner_xml_file:  return true");
  return true;
} //end load_inner_xml_file

////////////////////////////////////////////////////////////////////////////////
// Inner recursive function that loads each keyframe or file included in
// the original xml
////////////////////////////////////////////////////////////////////////////////
inline bool load_from_xml_file2(KeyframeGesture & gesture,
                                const std::string & xml_filename, const double startTime) {

  ROS_INFO("load_from_xml_file2:  %s",xml_filename.c_str());
  //create path
  std::string xml_filename_clean = vision_utils::replace_find_tags(xml_filename);
  // convert relative path to absolute if needed
  if (xml_filename_clean.empty() || xml_filename_clean[0] != '/')
    xml_filename_clean = gesture_synchronizer::gesture_files_folder() + xml_filename_clean;
  // add XML extension if needed
  if (xml_filename_clean.find(".xml") == std::string::npos)
    xml_filename_clean = xml_filename_clean + ".xml";
  // read xml
  vision_utils::XmlDocument doc;
  bool xml_read_success = doc.load_from_file(xml_filename_clean);
  if(!xml_read_success)
    return false;

  if(!load_gestures(doc, gesture, startTime))
    return false;
  if(!load_inner_xml_file(doc, gesture))
    return false;


  //ROS_INFO("load_from_xml_file2:  return true");
  return true;
} // end load_from_xml_file();

////////////////////////////////////////////////////////////////////////////////
// Outer function to be called from other parts of the gesture_synchronizer.
// It loads the complete gesture from an xml file (it can have includes too)
////////////////////////////////////////////////////////////////////////////////
inline bool load_from_xml_file(KeyframeGesture & gesture,
                               const std::string & xml_filename) {
  // clear old gesture
  gesture.keytimes.clear();
  gesture.joint_names.clear();
  gesture.joint_values.clear();

  if(!load_from_xml_file2(gesture,xml_filename,0))
    return false;

  sort_by_time(gesture);
  return true;
} // end load_from_xml_file();

////////////////////////////////////////////////////////////////////////////////

inline bool load_includes(vision_utils::XmlDocument & doc, KeyframeGesture & gesture) {
  std::vector<vision_utils::XmlDocument::Node*> files_nodes;
  doc.get_all_nodes_at_direction(doc.root(), "gestures.include", files_nodes);

  for (unsigned int key_idx = 0; key_idx < files_nodes.size(); ++key_idx) {
    // read attributes ==>
    //<include keytime=[double] file=[string] repetitions=[integer] gap=[double]>

    Time keytime =
        doc.get_node_attribute<Time>(files_nodes[key_idx], "keytime", 0);
    std::string fileName =
        doc.get_node_attribute<std::string>(files_nodes[key_idx], "file", "");
    int repetitions =
        doc.get_node_attribute<int>(files_nodes[key_idx], "repetitions", 1);
    double gap =
        doc.get_node_attribute<double>(files_nodes[key_idx], "gap", 0);

    //reformat data -- like gesture
    gesture.joint_names.push_back("include");
    gesture.keytimes.push_back(keytime);
    std::ostringstream joint_value;
    joint_value << fileName.c_str() << ";" << repetitions << ";" << gap;
    gesture.joint_values.push_back(joint_value.str());
  }

  return true;
}

////////////////////////////////////////////////////////////////////////////////

inline bool load_xml_labels(KeyframeGesture & gesture,
                            const std::string & xml_filename) {
  // clear old gesture
  gesture.keytimes.clear();
  gesture.joint_names.clear();
  gesture.joint_values.clear();

  // read xml
  vision_utils::XmlDocument doc;
  bool xml_read_success = doc.load_from_file(xml_filename);
  if(!xml_read_success)
    return false;

  if(!load_gestures(doc, gesture, 0))
    return false;
  if(!load_includes(doc, gesture))
    return false;

  //Sort all gesture by keytime
  sort_by_time(gesture);
  return true;
} // end load_from_xml_file();

////////////////////////////////////////////////////////////////////////////////

inline void save_to_xml_file_labeled(const KeyframeGesture & gesture,
                                     const std::string & xml_filename) {
  vision_utils::XmlDocument doc;
  vision_utils::XmlDocument::Node* keyframes_node = doc.add_node
      (doc.root(), "gestures", "");
  for (unsigned int key_idx = 0; key_idx < gesture.keytimes.size(); ++key_idx) {
    if(gesture.joint_names[key_idx] == "include"){
      vision_utils::XmlDocument::Node* include_node = doc.add_node
          (keyframes_node, "include", "");

      std::vector<std::string> values;
      vision_utils::StringSplit(gesture.joint_values[key_idx],";",  &values);
      if(values.size()!=3)
        break;

      //<include keytime=[double] file=[string] repetitions=[integer] gap=[double]>
      doc.set_node_attribute(include_node, "keytime", gesture.keytimes[key_idx]);
      doc.set_node_attribute(include_node, "file", values[0]);
      doc.set_node_attribute(include_node, "repetitions", values[1]);
      doc.set_node_attribute(include_node, "gap", values[2]);

    }
    else {
      vision_utils::XmlDocument::Node* keyframe_node = doc.add_node
          (keyframes_node, "gesture", "");
      doc.set_node_attribute(keyframe_node, "keytime", gesture.keytimes[key_idx]);
      doc.set_node_attribute(keyframe_node, "joint_name", gesture.joint_names[key_idx]);
      doc.set_node_attribute(keyframe_node, "joint_value", gesture.joint_values[key_idx]);
    }
  } // end loop key_idx

  // save the generated doc
  doc.write_to_file(xml_filename);
} // end save_to_xml_file();

} // end namespace gesture_io

#endif // GESTURE_IO_H
