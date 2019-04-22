<template>
  <div class="h-100 w-100 d-flex flex-column">
    <div
      class="btn-toolbar mb-1 w-100 d-flex flex-row-reverse"
      style="height:40px;"
      role="toolbar">
      <button
        type="button"
        class="btn btn-success h-100"
        style="align-items: center; width:125px; min-width:125px;"
        @click="addPatrolToPatrolList()">
        Save patrol
      </button>
      <div
        class="h-100 w-auto text-left mr-1"
        style="flex:1;">
        <input
          id="nameTextBox"
          v-model="newPatrolName"
          type="text"
          :placeholder="[[ placehold ]]"
          class="form-control h-100">
      </div>
    </div>
    <!-- Table -->
    <div
      class="w-100"
      style="height: calc(100% - 40px - 0.25rem)">
      <div class="mh-100 overflow-auto">
        <table
          id="saved-patrol-table"
          class="table table-borderless table-striped border-right border-left"
          style="text-align: center">
          <thead
            class="text-white bg-green-sb">
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
                class="w-50"
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
    <!--div
        class="btn-toolbar mb-1 w-100 d-flex flex-row-reverse"
        style="height:40px;"
        role="toolbar">
        <button
          type="button"
          class="btn btn-success h-100"
          style="align-items: center; width:125px; min-width:125px;"
          @click="addPatrolToPatrolList()">
          Save patrol
        </button>
        <div
          class="h-100 w-auto text-left mr-1"
          style="flex:1;">
          <input
            id="nameTextBox"
            v-model="newPatrolName"
            type="text"
            :placeholder="[[ placehold ]]"
            class="form-control h-100">
        </div>
      </div>
    </div-->
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
