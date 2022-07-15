/**rviz panel plugin for setting prediction time
 * publishes topic predicted_ogm_index (Int32)
 *
 * rviz plugin tutorial : http://docs.ros.org/kinetic/api/rviz_plugin_tutorials/html/panel_plugin_tutorial.html
 * qt5 tutorial         : http://zetcode.com/gui/qt5/
 */

#ifndef PG_EDITOR_PANEL_PG_EDITOR_PANEL_H_
#define PG_EDITOR_PANEL_PG_EDITOR_PANEL_H_

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>

#include <QGroupBox>

#include <QLabel>
#include <QLineEdit>
#include <QCheckBox>
#include <QSlider>
#include <QString>
#include <QPushButton>
#include <QComboBox>
#include <QSpinBox>
#include <QDoubleSpinBox>

#include <QKeyEvent>

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <rviz/panel.h>
// #include <rviz/render_panel.h>
#endif

#include <vector>
#include <std_srvs/Empty.h>

namespace pg_editor_panel
{
class PGEditorPanel : public rviz::Panel
{
  Q_OBJECT
public:
  PGEditorPanel(QWidget *parent = 0);

    virtual void load(const rviz::Config &config);
  virtual void save(rviz::Config config) const;

private Q_SLOTS:
  void saveData();
  void uploadNAS();
  void enumFactor();
  void optimize();
  void uploadReleasedNAS();
//   void loadData();

private:
  ros::NodeHandle nh;

  ros::ServiceClient save_client_;
  ros::ServiceClient bin_to_tree_client_;
  ros::ServiceClient upload_client_, upload_released_client_;
  ros::ServiceClient enum_factor_client, optimize_client;
  ros::ServiceClient client_export_graph;
  std::string area, proc_path;
  int zone,yymmdd, worker_id;
};

} // namespace spl_editor_panel

#endif