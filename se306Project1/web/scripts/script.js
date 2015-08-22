$(document).ready(function(){

    var lastUpdated = 0;
    var offlineCounter = 0; // we have a little counter to insure the connection didn't temporarily drop
    var online = false;
    var serverStatus = $('#server-status') // span id to display server status

    var table = $('#table').DataTable({
        ajax: '/ros/offline.json'
    });

    window.setInterval(function(){
        loadAsyncData()
    }, 1000);

    function loadAsyncData(){
        console.log("update:"+ new Date().getTime());
        console.log("online: "+ online);
        console.log("ModificationDate: "+ lastUpdated);

        checkIfServerIsOnline(); // we check when the json file was last updated to determine if the server is online

        if (online) {
            serverStatus.html('ROS Server is: ONLINE')
            table.ajax.url('/ros/state_file.json').load();
        }else{
            serverStatus.html('ROS Server is: OFFLINE')
            table.ajax.url('/ros/offline.json').load();
        }
    }

    function checkIfServerIsOnline(){
        $.get('/ros/state_file.json', function(result) {
            var time = result.lastModified;
            if (lastUpdated != time && lastUpdated != 0) {
                online = true
                offlineCounter = 0;
            } else {
                offlineCounter++;
            }

            if (offlineCounter > 4){
                online = false
                offlineCounter = 5;
            }
            lastUpdated = time;
        });
    }

    window.alert = function() {}; // dataTable throws crazy errors we need to throw so that updating doesn't stop

});