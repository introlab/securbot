<template>
  <div class="h-100 w-100 position-relative">
    <div class="h-25 w-100 position-relative">
      <button
        type="button"
        class="h-100 w-50"
        @click="addPatrolToPatrolList()">Add patrol</button>
    </div>
    <div class="h-75 overflow-auto position-relative">
      <table
        id="saved-patrol-table"
        class="table table-borderless table-striped border-left border-right">
        <thead class="text-white bg-green-sb">
          <th class="w-50">Name of the Patrol</th>
          <th class="w-25">Choose</th>
          <th class="w-25">Remove</th>
        </thead>
        <tbody>
          <!-- Ignore this "problem" -Edouard -->
          <tr
            v-for="(patrol,index) of patrolList"
            class="border-bottom">
            <td class="w-25">{{ patrol.Name }}</td>
            <td class="w-25">
              <button
                :id="'selectBtn'+index"
                type="button"
                class="btn btn-danger p-0 m-0 border border-secondary h-100 w-50"
                @click="selectPatrolFromList(index)">
                <img
                  src="~/open-iconic/svg/trash.svg"
                  alt=""
                  style="width:12px;height:12px;">
              </button>
            </td>
            <td class="w-25">
              <button
                :id="'removeBtn'+index"
                type="button"
                class="btn btn-danger p-0 m-0 border border-secondary h-100 w-50"
                @click="removePatrolFromList(index)">
                <img
                  src="~/open-iconic/svg/trash.svg"
                  alt=""
                  style="width:12px;height:12px;">
              </button>
            </td>
          </tr>
        </tbody>
      </table>
    </div>
  </div>
</template>

<script>
/*
* Author(s):  Valerie Gauthier <valerie.gauthier4@usherbrooke.ca>
*             
* File :  SaveLoad.vue
* Desc :  
*
* Dependencies :
*       
*       
*/


export default {
  name: 'save-load',
  props: ['waypointList', 'patrolList'],
  data() {
    return {
    };
  },
  methods: {
    removePatrolFromList(index) {
      this.patrolList.splice(index, 1);
    },
    addPatrolToPatrolList() {
      const patrol = [];
      patrol.Name = 'popo';
      patrol.waypoints = Array.from(this.waypointList);
      this.patrolList.push(patrol);
      console.log(this.patrolList);
    },
    
    selectPatrolFromList(index) {
      if (this.waypointList !== []) {
        console.log('overwrite plan');
        this.waypointList.splice(0, this.waypointList.length);
      } else {
        console.log(this.patrolList[index].waypoints);
      }
      console.log(this.patrolList[index]);
      this.patrolList[index].waypoints.forEach((element) => {
        this.waypointList.push(element);
      });
    },
  },
};
</script>

<style>
</style>
