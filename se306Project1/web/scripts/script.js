$(document).ready(function(){

    var data = [ ]

    var table = $('#table').DataTable({
        ajax: '/ros/state_file.json'
    });

    window.setInterval(function(){
        loadAsyncData()
    }, 5000);

    function loadAsyncData(){
        console.log("test")
        table.ajax.url( '/ros/state_file.json' ).load();
    }



});