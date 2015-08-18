$(document).ready(function(){

    var data = [ ]

    var table = $('#table').DataTable({
        ajax: '/ros/state_file.json'
    });

    window.setInterval(function(){
        loadAsyncData()
    }, 500);

    function loadAsyncData(){
        console.log("update:"+ new Date().getTime())
        table.ajax.url( '/ros/state_file.json' ).load();
    }

    window.alert = function() {}; // dataTable throws crazy errors we need to throw

});