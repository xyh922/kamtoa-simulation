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

        // Implementatiob of Q_SLOTS
        public Q_SLOTS:
            // Outmost poi service (for dropdown selection callback)
            void goToPoi(int poi_id_);
            void stopTraverse();

        protected Q_SLOTS:
            // Read the topic name from topmost textbox
            void getPOIList();
            void handlePOISelection();

        // Member variables
        protected:
            // One-line Textbox
            ros::ServiceClient  loadMapClient;
            ros::ServiceClient  gotoPoiClient;
            // Service slave
            kamtoa_map_manager::listPoi srv;
            // Node handle
            ros::NodeHandle     nh_;
            QListWidget         *listWidget;
            std::vector<kamtoa_map_manager::poi>   poi_list;
            int                 poi_id;

        private:
            void setupPOIList();


    };
}
