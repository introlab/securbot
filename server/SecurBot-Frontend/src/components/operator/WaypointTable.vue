<template>
  <div class="h-100">
    <div class="patrol-btn-container">
        <div class="title">Patrol :</div>
        <button class="btn btn-confirm" v-on:click="sendPatrol()">Confirm</button>
        <button class="btn btn-cancel" v-on:click="clearWaypointList()">Reset</button>
    </div>
    <div class="waypoint-list-container">
      <table id="waypoint-table" class="waypoint-list">
        <thead class="table-header">
          <th class="waypoint-cell">X</th>
          <th class="waypoint-cell">Y</th>
          <th class="waypoint-cell">Yaw</th>
          <th class="waypoint-cell">Remove</th>
        </thead>
        <tbody>
          <!-- Ignore this "problem" -Edouard -->
          <tr class="table-row" v-for="(waypoint,index) of waypointList">
            <td class="waypoint-cell">{{waypoint.x.toFixed(1)}}</td>
            <td class="waypoint-cell">{{waypoint.y.toFixed(1)}}</td>
            <td class="waypoint-cell">{{waypoint.yaw.toFixed(1)}}</td>
            <td class="waypoint-cell">
              <button class="removeBtn"
                v-bind:id="'removeBtn'+index" 
                v-on:click="removeWaypointFromList(index)"
              />
            </td>
          </tr>
        </tbody>
      </table>
    </div>
  </div>
</template>

<script>
export default {
  name:"waypoint-table",
  props:["waypointList", "bus"],
  data(){
    return{
    }
  },
  methods:{
    //Clear the list and empty the html table
    clearWaypointList(){
      this.waypointList.splice(0,this.waypointList.length);
    },
    //Remove waypoint from html table - ONLY FIRST LINE USE, TO FIX
    removeWaypointFromList(index){
      this.waypointList.splice(index,1);
      /*
      var table = document.getElementById("waypoint-table");
      var lengthTable = table.rows.length;
      var lengthWaypoints = this.waypointList.length;
      for(var i = index; i < lengthTable; i++)
      {
        var row = table.deleteRow(index);
      };
      
      for(var i = index-1; i < lengthWaypoints; i++)
      { 
        var wp = this.waypointList[i];
        this.addWaypointToList(wp);
      };  */ 
    },
    //Send the waypoint list for patrol scheduling
    sendPatrol(){
      this.bus.$emit('send-patrol', JSON.stringify(this.waypointList));
    }
  }
}
</script>

<style>
.patrol-btn-container{
  height: 10%;
  padding: 2px;
  align-content: center
}
.title{
  width: 75%;
  height: 100%;
  font-size: 20pt;
  float: left;
  text-align: left;
}
.btn{
  width: 25%;
  height: 50%;
  float: right;
}
.btn-confirm{
  background-color: #4CAF50;
  border: 1px solid dimgray;
  border-radius: 4px;

  -webkit-transition-duration: 0.4s; /* Safari */
  transition-duration: 0.4s;
}
.btn-cancel{
  background-color: rgb(221, 50, 50);
  border: 1px solid dimgray;
  border-radius: 4px;

  -webkit-transition-duration: 0.4s; /* Safari */
  transition-duration: 0.4s;
}
.waypoint-list-container{
  width: 100%;
  max-height: 40%;
  overflow: auto;
}
.waypoint-list{
  width: 100%;
  border-collapse: collapse;
  border-right: 1px solid #ddd;
  border-left: 1px solid #ddd;
}
.table-header{
  background-color: #4CAF50;
  color: white;
}
.table-row{
  border-bottom: 1px solid #ddd;
}
.table-row:nth-child(even) {
  background-color: #f2f2f2;
}
.waypoint-cell{
  width: 20%;
  border-radius: 0px;
}
.removeBtn{
  width: 20px;
  height: 20px;
  background-color: lightcoral;
  border: 1px solid dimgray;
  border-radius: 4px;

  -webkit-transition-duration: 0.4s; /* Safari */
  transition-duration: 0.4s;
}
.removeBtn:hover{
  background-color: darkred;
}
</style>