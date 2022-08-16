#include "pg_editor_panel/PGEditorPanel.h"
#include <pg_editor_panel/GetCalibration.h>

namespace pg_editor_panel
{
  PGEditorPanel::PGEditorPanel(QWidget *parent)
      : rviz::Panel(parent) //, dt_predict(1.0), predicted_ogm_info_valid(false)
  {
    ros::NodeHandle nh;
    ros::NodeHandle nh_pc("pg_editor_node");
    upload_client_ = nh.serviceClient<std_srvs::Empty>("/pg_editor_panel/upload_pc_tree");
    upload_released_client_ = nh.serviceClient<std_srvs::Empty>("/pg_editor_panel/upload_pc_tree_released");
    add_factor_client = nh.serviceClient<std_srvs::Empty>("/pg_editor_panel/add_factor");
    get_calibration_client = nh.serviceClient<pg_editor_panel::GetCalibration>("/pg_editor_panel/get_calibration");
    add_msg_pubs = nh.advertise<std_msgs::Empty>("/pg_editor_panel/add", 1);
    remove_msg_pubs = nh.advertise<std_msgs::Empty>("/pg_editor_panel/remove", 1);
    optimize_msg_pubs = nh.advertise<std_msgs::Empty>("/pg_editor_panel/optimize", 1);

    QHBoxLayout *mainLayout = new QHBoxLayout;
    QVBoxLayout *rightLayout = new QVBoxLayout;
    QVBoxLayout *leftLayout = new QVBoxLayout;

    QWidget *w = new QWidget(this);
    message1 = new QTextEdit(w);
    message2 = new QTextEdit(w);
    leftLayout->addWidget(message1);
    leftLayout->addWidget(message2);

    QPushButton *GetCalibrationButton = new QPushButton(tr("Get Calibration Result"));
    GetCalibrationButton->setStyleSheet("color: white; background-color: rgb(180,180,0); border-radius: 5px; height: 30px; margin-bottom: 5px;");
    connect(GetCalibrationButton, SIGNAL(clicked()), this, SLOT(getCalibration()));

    leftLayout->addWidget(GetCalibrationButton);

    QGroupBox *leftGroupBox = new QGroupBox("Input ref/in sensor name");
    leftGroupBox->setLayout(leftLayout);

    QGroupBox *rightGroupBox = new QGroupBox("Select Edge Mode");

    QPushButton *AddFactorButton = new QPushButton(tr("Add Factor"));
    AddFactorButton->setStyleSheet("color: white; background-color: rgb(180,0,180); border-radius: 5px; height: 30px; margin-bottom: 5px;");
    connect(AddFactorButton, SIGNAL(clicked()), this, SLOT(addFactor()));
    rightLayout->addWidget(AddFactorButton);

    QPushButton *RemoveButton = new QPushButton(tr("Remove Factor"));
    RemoveButton->setStyleSheet("color: white; background-color: rgb(0,180,180); border-radius: 5px; height: 30px; margin-bottom: 5px;");
    connect(RemoveButton, SIGNAL(clicked()), this, SLOT(removeFactor()));
    rightLayout->addWidget(RemoveButton);

    QPushButton *OptimizeButton = new QPushButton(tr("Optimize"));
    OptimizeButton->setStyleSheet("color: white; background-color: rgb(180,180,0); border-radius: 5px; height: 30px; margin-bottom: 5px;");
    connect(OptimizeButton, SIGNAL(clicked()), this, SLOT(optimize()));
    rightLayout->addWidget(OptimizeButton);

    rightGroupBox->setLayout(rightLayout);

    mainLayout->addWidget(leftGroupBox);
    mainLayout->addWidget(rightGroupBox);

    setLayout(mainLayout);
  }

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

  void PGEditorPanel::getCalibration()
  {
    pg_editor_panel::GetCalibration get_calibration_srv;
    // publish "/pg_editor_panel/add"
    std::string input_string = message1->toPlainText().toStdString();

    if(input_string=="") return;

    std::string delimiters = "-";
    boost::char_separator<char> sep(delimiters.c_str());
    boost::tokenizer<boost::char_separator<char>> tok(input_string, sep);
    boost::tokenizer<boost::char_separator<char>>::iterator itr = tok.begin();

    get_calibration_srv.request.sensor_ref = *itr++;
    get_calibration_srv.request.sensor_in = *itr++;

    if(itr!=tok.end()){
      message1->setPlainText("Wrong Calibration input");
      message2->clear();
      return;
    }

    get_calibration_client.call(get_calibration_srv);

    if(!get_calibration_srv.response.validate_sensor_name){
      message1->setPlainText("Wrong Calibration input");
      message2->clear();
      return;
    }

    std::string result_string = "", temp_string;
    for(int i=0; i<get_calibration_srv.response.calibration_result_vec.size(); i++){
      temp_string = std::to_string(get_calibration_srv.response.calibration_result_vec.at(i));
      result_string += temp_string+"\n";
    }
    message1->clear();
    message2->setPlainText(QString::fromStdString(result_string));
  }

  void PGEditorPanel::addFactor()
  {
    std_msgs::Empty empty_msg;
    add_msg_pubs.publish(empty_msg);
  }

  void PGEditorPanel::removeFactor()
  {
    std_msgs::Empty empty_msg;
    remove_msg_pubs.publish(empty_msg);
  }

  void PGEditorPanel::optimize()
  {
    std_msgs::Empty empty_msg;
    optimize_msg_pubs.publish(empty_msg);
  }
} // namespace pg_editor_panel

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(pg_editor_panel::PGEditorPanel, rviz::Panel)
