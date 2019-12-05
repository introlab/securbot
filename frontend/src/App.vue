<template>
  <!-- App -->
  <div
    id="app"
    class="h-100"
  >
    <!-- Router View to Layout -->
    <router-view />
  </div>
</template>

<script>
/**
 * The main/parent Vue component. Everything starts from here.
 * If unscoped CSS are declared here, it will affect all children.
 *
 * Note: If the size of the screen of the user is small enough to be considered a mobile device,
 *    the main component force redirect the user to the teleop view.
 *
 * @module App
 * @version 1.0.0
*/

export default {
  name: 'app',
  data() {
    return {
      mobileDeviceCheck: '',
    };
  },
  /**
   * After the component is mounted (just before rendering) the database is read.
   *
   * Note: This is the only time the UI will check for robot datas on the database, an option
   * should be added to either allow the user to refresh all database datas or have an interval
   * that refresh those automatically. It could also be done through the websocket of the database.
   */
  mounted() {
    this.$store.dispatch('database/initLocalData');
    this.mobileDeviceCheck = setInterval(() => {
      if (document.documentElement.clientWidth < 576 && !this.$route.path.includes('teleop')) {
        this.$router.push('teleop');
      }
    }, 1000);
  },
};
</script>

<style>
</style>
