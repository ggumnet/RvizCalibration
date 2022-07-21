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
  add_factor_client      = nh.serviceClient<std_srvs::Empty>("/pg_editor_panel/add_factor");
  optimize_client         = nh.serviceClient<std_srvs::Empty>("/pg_editor_panel/optimize");
  add_msg_pubs = nh.advertise<std_msgs::Empty>("/pg_editor_panel/add", 1);
  remove_msg_pubs = nh.advertise<std_msgs::Empty>("/pg_editor_panel/remove", 1);

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
  QHBoxLayout *mainLayout = new QHBoxLayout;
  QVBoxLayout *rightLayout = new QVBoxLayout;
  QVBoxLayout *leftLayout = new QVBoxLayout;



  /*left layout*/

  QGroupBox *leftGroupBox = new QGroupBox("Transformation result");

  /*left layout*/

  // QTableView *view = new QTableView;

  // QAbstractItemModel *model;


  // model->setHeaderData(0, Qt::Horizontal, QObject::tr("Data type"));
  // model->setHeaderData(1, Qt::Horizontal, QObject::tr("Result"));

  // view->setModel(model);
  // view->show();

  // view->setEditTriggers(QAbstractItemView::NoEditTriggers);


  QTableWidget *table = new QTableWidget(this);
  table->setRowCount(7);
  table->setColumnCount(1);

  QStringList vLabels;
  vLabels << "tx" << "ty" << "tz" << "qx" << "qy" << "qz" << "qw";
  
  table -> setVerticalHeaderLabels(vLabels);

  leftLayout->addWidget(table);
  leftGroupBox->setLayout(leftLayout);

  /*right layout 1*/

  // QGroupBox *rightGroupBox = new QGroupBox("Select Edge Mode");

  // QRadioButton *radio1 = new QRadioButton(tr("Get Transformation"));
  // QRadioButton *radio2 = new QRadioButton(tr("Modify Edge"));

  // radio1->setChecked(true);

  // rightLayout -> addWidget(radio1);
  // rightLayout -> addWidget(radio2);

  // rightGroupBox->setLayout(rightLayout);


  /* right layout 2*/

  QGroupBox *rightGroupBox = new QGroupBox("Select Edge Mode");

  QPushButton *AddFactorButton = new QPushButton(tr("Add Factor"));
  AddFactorButton->setStyleSheet("color: white; background-color: rgb(180,0,180); border-radius: 5px; height: 30px; margin-bottom: 5px;");
  connect(AddFactorButton, SIGNAL(clicked()), this, SLOT(addFactor()));

  rightLayout->addWidget(AddFactorButton);

  QPushButton *OptimizeButton = new QPushButton(tr("Remove Factor"));
  OptimizeButton->setStyleSheet("color: white; background-color: rgb(0,180,180); border-radius: 5px; height: 30px; margin-bottom: 5px;");
  connect(OptimizeButton, SIGNAL(clicked()), this, SLOT(removeFactor()));

  rightLayout->addWidget(OptimizeButton);
  rightGroupBox->setLayout(rightLayout);

  mainLayout->addWidget(leftGroupBox);
  mainLayout->addWidget(rightGroupBox);

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


void PGEditorPanel::addFactor()
{
  // std_srvs::Empty empty_srv;
  // if(add_factor_client.call(empty_srv)){

  // }else{
  //   ROS_ERROR("Failed to add factor");
  // }
  std_msgs::Empty empty_msg;
  //publish "/pg_editor_panel/add"
  add_msg_pubs.publish(empty_msg);
}
void PGEditorPanel::removeFactor(){
  std_msgs::Empty empty_msg;
  //publish "/pg_editor_panel/remove"
  remove_msg_pubs.publish(empty_msg);
}

} // namespace pg_editor_panel

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(pg_editor_panel::PGEditorPanel, rviz::Panel)
