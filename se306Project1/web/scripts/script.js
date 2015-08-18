$(document).ready(function(){
    $.get('/ros/status.txt', function(result) {
        table = $('#table').DataTable();
        var entities = JSON.parse(result)
        if (result == 'ON') {
            alert('ON');
        } else if (result == 'OFF') {
            alert('OFF');
        } else {
            alert(result);
        }
    });
});