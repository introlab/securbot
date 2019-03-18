<template>
    <b-jumbotron id="patrol-layout" class="h-100 w-100" :fluid="true" :container-fluid="true" bg-variant="light">
      <b-row class="h-100">
        <b-col md="6" class="mh-100">
          <waypoint-table :waypointList="test"/>
        </b-col>
        <b-col md="6" class="mh-100">
          <patrol-map :waypointList="test" :bus="bus"/>
        </b-col>        
      </b-row>
      <!--<waypoint mapId='map' showMap='true' :bus='bus'/>-->
    </b-jumbotron>
</template>


<script>
import PatrolMap from "../operator/PatrolMap.vue";
import WaypointTable from "../operator/WaypointTable.vue";

export default {
  name:'patrol-page',
  props:['bus'],
  components:{
    PatrolMap,
    WaypointTable,
  },
  data(){
    return{
      test:[],
    }
  }
}
</script>

<style>

</style>

