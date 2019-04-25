<template>
  <!-- SaveLoad widget -->
  <div class="h-100 w-100 d-flex flex-column">
    <!-- Header container -->
    <div
      class="btn-toolbar mb-1 w-100 d-flex flex-row-reverse"
      style="height:40px;"
      role="toolbar">
      <!-- Save patrol button -->
      <button
        type="button"
        class="btn btn-success h-100"
        style="align-items: center; width:125px; min-width:125px;"
        @click="addPatrolToPatrolList()">
        Save patrol
      </button>
      <!-- Patrol name input -->
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
    <!-- Patrol List container -->
    <div
      class="w-100"
      style="height: calc(100% - 40px - 0.25rem)">
      <!-- Overflow container -->
      <div class="mh-100 overflow-auto">
        <!-- Patrol table -->
        <table
          id="saved-patrol-table"
          class="table table-borderless table-striped border-right border-left"
          style="text-align: center">
          <!-- Header -->
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
          <!-- Table body -->
          <tbody>
            <!-- Table row per patrol in list -->
            <tr
              v-for="(patrol,index) of patrolList"
              :key="patrol.Name"
              class="border-bottom">
              <!-- Patrol Name -->
              <td
                class="w-50"
                style="text-align:left">
                {{ patrol.Name }}
              </td>
              <!-- Select button -->
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
              <!-- Remove button -->
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
  </div>
</template>

<script>
/**
 * Complementary module allowing to send a list of patrols (a patrol being a name
 * and a list of waypoint) to the server and manage the list form the user
 * interface application.The table contains two colomn of buttons use to delete the row they
 * are in and resend new version of the modified patrol list. The other button is used to
 * copy the waypoints of a saved patrol to the current waypoint list.
 * This component have the following dependency :
 * Bootstrap-Vue for styling.
 *
 *
 * @module widget/SaveLoad
 * @vue-prop {Object[]} waypointList - Lists the current waypoints
 * @vue-prop {Object[]} patrolList - Lists the saved waypoint lists
 * @vue-data {String} newPatrolName - Name of the current waypoint list
 * @vue-data {String} placehold - Message of the placeholder for name input textbox
 * /

/** Disabled comment documentation
 * Might use those eventually by forking jsdoc-vue-js so it can manage the author
 * and version tag correctly
 * @author Valerie Gauthier <valerie.gauthier@usherbrooke.ca>
 * @version 1.0.0
 */

export default {
  name: 'save-load',
  props: {
    waypointList: {
      type: Array,
      default: () => [],
      required: true,
    },
    patrolList: {
      type: Array,
      default: () => [],
      required: true,
    },
  },
  data() {
    return {
      newPatrolName: '',
      placehold: 'Patrol name',
    };
  },
  methods: {
    /**
     * Remove the patrol from the list, eventually this will remove on DB.
     * @method
     * @param {Number} index - Index of the element to remove.
     */
    removePatrolFromList(index) {
      this.patrolList.splice(index, 1);
    },
    /**
     * Add a patrol to the list, eventually will add on DB.
     * @method
     */
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
    /**
     * Load the selected patrol from the list.
     * @method
     * @param {Number} index - Index of the element to load.
     */
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
