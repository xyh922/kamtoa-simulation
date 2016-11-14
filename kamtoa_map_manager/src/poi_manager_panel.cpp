/******************************
 *  POI Manager Panel
 *  Map POI Manager widget for RVIZ (Qt5)
 *  - Usage : GUI For sending goal to robot
  from RVIZ Gui
 *  Author : Theppasith Nisitsukcharoen
 *  09-Nov-2016
 *******************************/

#include <stdio.h>

#include <iostream>
#include <iterator>
#include <std_msgs/Empty.h>
#include "kamtoa_map_manager/poi_manager_panel.hpp"
#include <pluginlib/class_list_macros.h>

namespace kamtoa_map_manager{

        // Class Constructor
        POIPanel::POIPanel(QWidget *parent)
        : rviz::Panel(parent)
        , poi_id(-1)
        {
            // Service to link with the poiManager Node
            loadMapClient = nh_.serviceClient<kamtoa_map_manager::listPoi>("/get_poi_list");
            gotoPoiClient = nh_.serviceClient<kamtoa_map_manager::gotoPoi>("/goto_poi");

            // Stop Traversing Publisher
            cancelGoalPub = nh_.advertise<std_msgs::Empty>("/kamtoa/cancel",1);

            // Setup UI
            setupUI();

            // Load the POI List from ROS Service
            setupPOIList();

        }

        void POIPanel::setupUI(){
          // Upper most layout for waypoint path display.
          QHBoxLayout *path_pane     = new QHBoxLayout;
          QLabel *path_text          = new QLabel("poi status : ");
          path_label                 = new QLabel(this);
          path_pane->addWidget(path_text);
          path_pane->addWidget(path_label);

          // Create top Layout for Operate plugin
          QHBoxLayout *top_pane      = new QHBoxLayout;
          QPushButton *reloadButton  = new QPushButton(tr("Reload"));
          label                      = new QLabel(this);
          top_pane->addWidget(reloadButton);
          top_pane->addWidget(label);

          // Create List H-layout
          QHBoxLayout *layout_list   = new QHBoxLayout;
          // List for displaying POI String
          listWidget                 = new QListWidget(this);
          layout_list->addWidget(listWidget);

          // Create Button H-layout
          QHBoxLayout *layout_button = new QHBoxLayout;
          QPushButton *goButton      = new QPushButton(tr("Go"));
          QPushButton *stopButton    = new QPushButton(tr("Stop"));
          // Add Buttons
          layout_button->addWidget(goButton);
          layout_button->addWidget(stopButton);

          // Main Layout
          QVBoxLayout* layout       = new QVBoxLayout;
          layout->addLayout( path_pane );
          layout->addLayout( top_pane );
          layout->addLayout( layout_list );
          layout->addLayout( layout_button );
          setLayout( layout );

          // Connect QT Event
          connect( listWidget,
            SIGNAL(itemSelectionChanged()),
            this, SLOT( handlePOISelection()));
          connect( goButton,
              SIGNAL(clicked()),
              this, SLOT( goToPoi()));
          connect( reloadButton,
              SIGNAL(clicked()),
              this, SLOT( handleReloading()));

          connect( stopButton,
              SIGNAL(clicked()),
              this, SLOT( handleStop()));

        }

        void POIPanel::load( const rviz::Config& config )
        {
            rviz::Panel::load( config );
        }
        void POIPanel::save( rviz::Config config ) const
        {
            rviz::Panel::save( config );
        }

        void POIPanel::getPOIList(){
          // Call ROS Service (/get_poi_list)
          if (loadMapClient.call(srv))
          {
              std::vector<kamtoa_map_manager::poi> poi_list_receive(srv.response.poi_list);
              poi_list = poi_list_receive;
              // Clear List first
              listWidget->clear();
              // Insert fetched data into listWidget as listWidget item
              for(int i = 0 ; i < poi_list.size() ; i++){
                std::string test = poi_list[i].poi_name;
                QListWidgetItem *newItem = new QListWidgetItem;
                QString qstr = QString::fromStdString(test);
                newItem->setText(qstr);
                listWidget->insertItem(i, newItem);
              }
              path_string = srv.response.file_path;
          }else{
              // Cannot get POI List through services
              listWidget->clear();
              path_string = "No POIManager Service Online !";
          }
        }

        void POIPanel::setupPOIList(){
            // Fetch POI from ROS Service
            getPOIList();
            // Clear current POI Label and POI File Label Selection
            label->clear();
            path_label->clear();
            // Set the label for file path
            if(path_string != ""){
                QString path_qstring = QString::fromStdString(path_string);
                path_label->setText(path_qstring);
            }else{
                QString qstr = QString::fromStdString("No file selected !");
                path_label->setText(qstr);
            }
            // Reset the current going poi_id
            poi_id = -1;
        }

        void POIPanel::handleReloading(){
            setupPOIList();
        }

        void POIPanel::handlePOISelection(){
          // Event to handle user clicked item in the listWidget
            std::string selectedText = listWidget->currentItem()->text().toStdString();
            int selectedPoiIndex = listWidget->row(listWidget->currentItem());
            label->setText(listWidget->currentItem()->text());
            poi_id = selectedPoiIndex;
        }

        void POIPanel::goToPoi(){
            // Call Go Poi Service
            srv_gotopoi.request.id = poi_id;
            if (gotoPoiClient.call(srv_gotopoi))
            {
                // Success callbacks
            }
            else{
                // Error Callbacks
            }
        }

        void POIPanel::handleStop(){
            stopTraverse();
        }

        void POIPanel::stopTraverse(){
            cancelGoalPub.publish(*new std_msgs::Empty());
        }
}
PLUGINLIB_EXPORT_CLASS(kamtoa_map_manager::POIPanel,rviz::Panel);
