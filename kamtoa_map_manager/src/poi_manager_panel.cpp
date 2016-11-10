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


#include "kamtoa_map_manager/poi_manager_panel.hpp"
#include <pluginlib/class_list_macros.h>

namespace kamtoa_map_manager{

        // Class Constructor
        POIPanel::POIPanel(QWidget *parent)
        : rviz::Panel(parent)
        , poi_id(-1)
        {

            loadMapClient = nh_.serviceClient<kamtoa_map_manager::listPoi>("/get_poi_list");
            gotoPoiClient = nh_.serviceClient<kamtoa_map_manager::gotoPoi>("/goto_poi");

            // Create List H-layout
            QHBoxLayout *layout_list   = new QHBoxLayout;
            // List for displaying POI String
            listWidget     = new QListWidget(this);
            layout_list->addWidget(listWidget);

            // Create Button H-layout
            QHBoxLayout *layout_button   = new QHBoxLayout;
            QPushButton *goButton = new QPushButton(tr("Go"));;
            QPushButton *stopButton = new QPushButton(tr("Stop"));
            // Add Buttons
            layout_button->addWidget(goButton);
            layout_button->addWidget(stopButton);

            setupPOIList();
            // Main Layout
            QVBoxLayout* layout = new QVBoxLayout;
            layout->addLayout( layout_list );
            layout->addLayout( layout_button );
            setLayout( layout );


            connect( listWidget,
              SIGNAL(itemSelectionChanged()),
              this, SLOT( handlePOISelection()));
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
          // Call Service!
          if (loadMapClient.call(srv))
          {
              std::vector<kamtoa_map_manager::poi> poi_list_receive(srv.response.poi_list);
              poi_list = poi_list_receive;
              for(int i = 0 ; i < poi_list.size() ; i++){
                std::string test = poi_list[i].poi_name;
                QListWidgetItem *newItem = new QListWidgetItem;
                QString qstr = QString::fromStdString(test);
                newItem->setText(qstr);
                listWidget->insertItem(i, newItem);
              }
          }
        }

        void POIPanel::setupPOIList(){
            getPOIList();
        }

        void POIPanel::handlePOISelection(){
            listWidget->currentItem();
            std::cout << "HI" << std::endl;
        }









}
PLUGINLIB_EXPORT_CLASS(kamtoa_map_manager::POIPanel,rviz::Panel);
