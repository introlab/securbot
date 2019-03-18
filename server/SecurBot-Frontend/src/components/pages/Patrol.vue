<template>
    <b-jumbotron id="patrol-layout" :fluid="true" :container-fluid="true" bg-variant="light">
      <waypoint mapId='map' showMap='true' :bus='bus'/>
    </b-jumbotron>
</template>


<script>
import Waypoint from "../operator/Waypoint.vue";

export default {
  name:'patrol-page',
  props:['bus'],
  components:{
    Waypoint
  },
  data(){
    return{
    }
  }
}
</script>

<style>
/*
.patrol-layout{
  height: 700px;
  width: 100%;
  margin: auto;
}
*/
</style>

