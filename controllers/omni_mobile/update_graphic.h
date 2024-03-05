#ifndef MARKER_H
#define MARKER_H
#include <cassert>
#include <webots/supervisor.h>

WbNodeRef vpoint;
string INDI;

void update_self_marker(string DEF_PREFIX, int self_id, int self_team){

	vpoint = wb_supervisor_node_get_from_def("VPOINT");
	// double default_upper[3] = {0, 0, 39};
	// wb_supervisor_field_set_sf_vec3f(wb_supervisor_node_get_field(vpoint, "position"), default_upper);
	// wb_supervisor_node_move_viewpoint(wb_supervisor_node_get_from_def("SOCCER_FIELD"));


  	string MARKER_GROUP = DEF_PREFIX + ".NAIYOU.BASE_MOBILE_SHAPE.MARKERS.ID";
  	string ORI_ID = MARKER_GROUP+".ID_DIR_N";

  	INDI =  DEF_PREFIX + ".INDICATOR";

  	string ID_SHAPE_S = MARKER_GROUP+".ID_SHAPE_S";
  	string ID_SHAPE_E = MARKER_GROUP+".ID_SHAPE_E";
  	string ID_SHAPE_W = MARKER_GROUP+".ID_SHAPE_W";
  	string ID_SHAPE_MID = MARKER_GROUP+".ID_SHAPE_MID";

	wb_supervisor_node_set_visibility(wb_supervisor_node_get_from_def(ORI_ID.c_str()) , vpoint, 1);

	// if (self_team == 500) wb_supervisor_node_set_visibility(wb_supervisor_node_get_from_def(INDI.c_str()) , vpoint, 0);


	if (CHECK_BIT(self_id, 0)) 
		wb_supervisor_node_set_visibility(wb_supervisor_node_get_from_def(ID_SHAPE_MID.c_str()) , vpoint, 1);
	else 
		wb_supervisor_node_set_visibility(wb_supervisor_node_get_from_def(ID_SHAPE_MID.c_str()) , vpoint, 0);

	if (CHECK_BIT(self_id, 1)) 
		wb_supervisor_node_set_visibility(wb_supervisor_node_get_from_def(ID_SHAPE_E.c_str()) , vpoint, 1);
	else 
		wb_supervisor_node_set_visibility(wb_supervisor_node_get_from_def(ID_SHAPE_E.c_str()) , vpoint, 0);

	if (CHECK_BIT(self_id, 2)) 
		wb_supervisor_node_set_visibility(wb_supervisor_node_get_from_def(ID_SHAPE_S.c_str()) , vpoint, 1);
	else 
		wb_supervisor_node_set_visibility(wb_supervisor_node_get_from_def(ID_SHAPE_S.c_str()) , vpoint, 0);

	if (CHECK_BIT(self_id, 3)) 
		wb_supervisor_node_set_visibility(wb_supervisor_node_get_from_def(ID_SHAPE_W.c_str()) , vpoint, 1);
	else 
		wb_supervisor_node_set_visibility(wb_supervisor_node_get_from_def(ID_SHAPE_W.c_str()) , vpoint, 0);



	// string SELF_DEF_NAME =  wb_supervisor_node_get_def(wb_supervisor_node_get_self());
	// cout << SELF_DEF_NAME << '\n';
}


#endif
