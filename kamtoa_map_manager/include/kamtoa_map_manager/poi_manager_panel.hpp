/******************************
 *  POI Manager Panel
 *  Map POI Manager widget for RVIZ (Qt5)
 *  - Usage : GUI For sending goal to robot
  from RVIZ Gui
 *  Author : Theppasith Nisitsukcharoen
 *  09-Nov-2016
 *******************************/

#include <ros/ros.h>
#include <rviz/panel.h>
#include <kamtoa_map_manager/poi.h>
#include <kamtoa_map_manager/listPoi.h>
#include <kamtoa_map_manager/gotoPoi.h>
#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>
#include <QComboBox>
#include <QTreeWidget>
#include <QPushButton>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QListWidget>

namespace kamtoa_map_manager{

    class POIPanel: public rviz::Panel {
        Q_OBJECT
        public:
            // Create panel which bind to parent widget parameter (default id : 0)
            POIPanel(QWidget *parent = 0);
            virtual void load(const rviz::Config &config);
            virtual void save(rviz::Config config) const;

        // Implementation of Q_SLOTS
        public Q_SLOTS:
            // These function handle the outGoing event from plugin
            void goToPoi();
            void stopTraverse();

        protected Q_SLOTS:
            // These function handle the event callback from GUI
            void getPOIList();
            void handlePOISelection();
            void handleReloading();
            void handleStop();

        // Member variables
        protected:
            // One-line Textbox
            ros::ServiceClient  loadMapClient;
            ros::ServiceClient  gotoPoiClient;
            // ListPOI Service slave
            kamtoa_map_manager::listPoi srv;
            // GoToPOI Service slave
            kamtoa_map_manager::gotoPoi srv_gotopoi;
            // ROS Node handle
            ros::NodeHandle     nh_;
            // Cancel Goal Publisher for GUI
            ros::Publisher      cancelGoalPub;
            // QT Widget
            QListWidget         *listWidget;
            QLabel              *label; // label for selected poi
            QLabel              *path_label; // label for selected path
            // array to hold poi_list
            std::vector<kamtoa_map_manager::poi>   poi_list;
            int                 poi_id;
            std::string         path_string;

        private:
          // Initialize function here
            void setupPOIList();
            void setupUI();
    };
}
