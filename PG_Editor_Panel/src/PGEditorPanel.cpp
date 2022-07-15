#include "pg_editor_panel/PGEditorPanel.h"

namespace pg_editor_panel
{
PGEditorPanel::PGEditorPanel(QWidget *parent)
    : rviz::Panel(parent) //, dt_predict(1.0), predicted_ogm_info_valid(false)
{
  ros::NodeHandle nh;
  ros::NodeHandle nh_pc("pg_editor_node");
  save_client_            = nh.serviceClient<std_srvs::Empty>("/pg_editor_panel/save_pc");
  upload_client_          = nh.serviceClient<std_srvs::Empty>("/pg_editor_panel/upload_pc_tree");
  upload_released_client_ = nh.serviceClient<std_srvs::Empty>("/pg_editor_panel/upload_pc_tree_released");
  enum_factor_client      = nh.serviceClient<std_srvs::Empty>("/pg_editor_panel/enum_factor");
  optimize_client         = nh.serviceClient<std_srvs::Empty>("/pg_editor_panel/optimize");

  // if (!nh.param<int>("/yymmdd", yymmdd, 0)){
  //   ROS_ERROR("[PGEditorPanel] Failed to read param yymmdd");
  // }
  // if (!nh.param<int>("/Zone", zone, 0)){
  //   ROS_ERROR("[PGEditorPanel] Failed to read param Zone");
  // }
  // if (!nh.param<std::string>("/Area", area, "")){
  //   ROS_ERROR("[PGEditorPanel] Failed to read param Area");
  // }
  // if (!nh.param<std::string>("/root_path", proc_path, "")){
  //   ROS_ERROR("[PGEditorPanel] Failed to read param proc_path");
  // }

  QVBoxLayout *mainLayout = new QVBoxLayout;

  QPushButton *EnumFactorButton = new QPushButton(tr("Enum Factor"));
  EnumFactorButton->setStyleSheet("color: white; background-color: rgb(180,0,180); border-radius: 5px; height: 30px; margin-bottom: 5px;");
  connect(EnumFactorButton, SIGNAL(clicked()), this, SLOT(enumFactor()));

  mainLayout->addWidget(EnumFactorButton);

  QPushButton *OptimizeButton = new QPushButton(tr("Optimize"));
  OptimizeButton->setStyleSheet("color: white; background-color: rgb(0,180,180); border-radius: 5px; height: 30px; margin-bottom: 5px;");
  connect(OptimizeButton, SIGNAL(clicked()), this, SLOT(optimize()));

  mainLayout->addWidget(OptimizeButton);


  setLayout(mainLayout);

}

//

void PGEditorPanel::load(const rviz::Config &config)
{
  rviz::Panel::load(config);
}

void PGEditorPanel::save(rviz::Config config) const
{
  rviz::Panel::save(config);
}

void PGEditorPanel::uploadNAS()
{
  std_srvs::Empty empty_srv;
  upload_client_.call(empty_srv); 
}

void PGEditorPanel::uploadReleasedNAS()
{
  std_srvs::Empty empty_srv;
  upload_released_client_.call(empty_srv); 
}


void PGEditorPanel::saveData()
{
  std_srvs::Empty empty_srv;
  save_client_.call(empty_srv);
}


void PGEditorPanel::enumFactor()
{
  std_srvs::Empty empty_srv;
  if(enum_factor_client.call(empty_srv)){

  }else{
    ROS_ERROR("Failed to enum factor");
  }
}
void PGEditorPanel::optimize(){
  std_srvs::Empty empty_srv;
  if(optimize_client.call(empty_srv)){

  }else{
    ROS_ERROR("Failed to optimize");
  }
}

} // namespace pg_editor_panel

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(pg_editor_panel::PGEditorPanel, rviz::Panel)
