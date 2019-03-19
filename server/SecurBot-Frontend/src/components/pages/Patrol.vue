<template>
    <b-jumbotron id="patrol-layout" class="h-100 w-100" :fluid="true" :container-fluid="true" bg-variant="light">
      <b-row class="h-100">
        <b-col md="4" class="mh-100">
          <div class="align-middle p-1" style="height:10%">
            <div class="w-75 h-100 text-left float-left" style="font-size: 20pt">Patrol :</div>
            <button type="button" class="btn btn-success w-25 h-50" v-on:click="sendPatrol()">Confirm</button>
            <button type="button" class="btn btn-danger w-25 h-50" v-on:click="clearWaypointList()">Reset</button>
          </div>
          <div class="h-50">
            <waypoint-table :waypointList="waypointList"/>
          </div>
          <div>
          </div> 
        </b-col>
        <b-col md="8" class="mh-100">
          <patrol-map :waypointList="waypointList" :bus="bus"/>
        </b-col>        
      </b-row>
    </b-jumbotron>
</template>


<script>
import PatrolMap from "../widget/PatrolMap.vue";
import WaypointTable from "../widget/WaypointTable.vue";

export default {
  name:'patrol-page',
  props:['bus'],
  components:{
    PatrolMap,
    WaypointTable,
  },
  data(){
    return{
      waypointList:[],
    }
  },
  methods:{
    //Send the waypoint list for patrol scheduling
    sendPatrol(){
      this.bus.$emit('send-patrol', JSON.stringify(this.waypointList));
    },
    //Clear the list and empty the html table
    clearWaypointList(){
      this.waypointList = [];
    },
  }
}
</script>

<style>

</style>

