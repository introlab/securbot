<template>
  <div class="w-100 h-100">
    <div class="mh-100 overflow-auto">
      <table
        id="waypoint-table"
        class="table table-borderless table-striped border-left border-right">
        <thead class="text-white bg-green-sb">
          <th class="w-25">X</th>
          <th class="w-25">Y</th>
          <th class="w-25">Yaw</th>
          <th class="w-25">Remove</th>
        </thead>
        <tbody>
          <!-- Ignore this "problem" -Edouard -->
          <tr
            v-for="(waypoint,index) of waypointList"
            class="border-bottom">
            <td class="w-25">{{ waypoint.x.toFixed(1) }}</td>
            <td class="w-25">{{ waypoint.y.toFixed(1) }}</td>
            <td class="w-25">{{ waypoint.yaw.toFixed(1) }}</td>
            <td class="w-25">
              <button
                :id="'removeBtn'+index"
                type="button"
                class="btn btn-danger p-0 m-0 border border-secondary h-100 w-50"
                @click="removeWaypointFromList(index)">
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
* Author(s):  Edouard Legare <edouard.legare@usherbrooke.ca>,
*             Valerie Gauthier <valerie.gauthier4@usherbrooke.ca>
* File :  WaypointTable.vue
* Desc :  Vue SFC used as a widget to show waypoints in a array given as props.
*         The waypoints are shown in a html table generated with vue v-for.
*         The table contains a colomn of buttons use to delete the row they
*         are in and, at the same time, delete the corresponding waypoint in
*         the array.
*
* Dependencies :
*       -Bootstrap-Vue
*
*/

export default {
  name: 'waypoint-table',
  props: ['waypointList'],
  data() {
    return {
    };
  },
  methods: {
    // Remove waypoint from html table - ONLY FIRST LINE USE, TO FIX
    removeWaypointFromList(index) {
      this.waypointList.splice(index, 1);
    },
  },
};
</script>

<style>
.bg-trash{
  display: block;
  background: url("~/open-iconic/svg/trash.svg");
  fill: black;
}
</style>
