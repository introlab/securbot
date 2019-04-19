<template>
  <div class="h-100 w-100 position-relative">
    <div
      class="w-100 position-relative"
      style="height:12%; margin-bottom:2px; margin-top:2px">
      <button
        type="button"
        class="btn btn-success h-100 w-25"
        style="font-size: 2vmin; align-items: center; padding:0px; position:relative"
        @click="addPatrolToPatrolList()">
        Save patrol
      </button>
      <div
        class="h-100 w-75 position-relative text-left float-left"
        style="font-size: 2.2vh; padding-right:2x">
        <input
          id="nameTextBox"
          v-model="newPatrolName"
          :placeholder="[[ placehold ]]"
          class="h-100 w-100">
      </div>
    </div>
    <div
      class="overflow-auto position-relative">
      <table
        id="saved-patrol-table"
        class="table table-borderless table-striped border-right"
        style="height:90%; text-align:center">
        <thead
          class="text-white bg-green-sb"
          style="padding:0px">
          <th
            class="w-50"
            style="text-align:left">
            Patrols
          </th>
          <th class="w-25">
            Choose
          </th>
          <th class="w-25">
            Remove
          </th>
        </thead>
        <tbody>
          <tr
            v-for="(patrol,index) of patrolList"
            :key="patrol.Name"
            class="border-bottom">
            <td
              class="w-25"
              style="text-align:left">
              {{ patrol.Name }}
            </td>
            <td class="w-25">
              <button
                :id="'selectBtn'+index"
                type="button"
                class="btn btn-success p-0 m-0 border border-secondary h-100 w-50"
                @click="selectPatrolFromList(index)">
                <img
                  src="~/open-iconic/svg/check.svg"
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
* Desc :  Complementary module allowing to send a list of patrols (a patrol being a name
*         and a list of waypoint) to the server and manage the list form the user
*         interface application.The table contains two colomn of buttons use to delete the row they
*         are in and resend new version of the modified patrol list. The other button is used to
*         copy the waypoints of a saved patrol to the current waypoint list.
*
* Dependencies :
*       - Bootstrap.vue
*
*/

export default {
  name: 'save-load',
  props: ['waypointList', 'patrolList'],
  data() {
    return {
      newPatrolName: '',
      placehold: 'Patrol name',
    };
  },
  methods: {
    removePatrolFromList(index) {
      this.patrolList.splice(index, 1);
    },
    addPatrolToPatrolList() {
      const patrol = [];
      if (this.newPatrolName === '') {
        this.placehold = 'Enter name';
      } else {
        patrol.Name = this.newPatrolName;
        patrol.waypoints = Array.from(this.waypointList);
        this.patrolList.push(patrol);
        this.newPatrolName = '';
        this.placehold = 'Patrol name   ';
        console.log(this.patrolList);
      }
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
