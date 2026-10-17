from encounter_check import motion_audit


def test_holds_do_not_reset_at_repeated_zero_commands_and_turning_is_motion():
    a=motion_audit([(0,0,0),(5,0,0),(10,0,0),(20,0,.2),(21,0,0)],[(12,1),(20.5,2)],0,25,1e-6,1e-6)
    assert a['longest_zero_command_s']==20
    assert a['contacts_while_commanded_motion']==1
    assert a['ego_contacts'][0]['moving'] is False


def test_prestart_command_and_final_hold_are_counted():
    a=motion_audit([(0,.1,0),(3,0,0)],[],2,22,1e-6,1e-6)
    assert a['longest_zero_command_s']==19
