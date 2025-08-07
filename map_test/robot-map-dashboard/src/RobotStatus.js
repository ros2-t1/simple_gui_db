import React from 'react';
import { Paper, Typography, List, ListItem, ListItemIcon, ListItemText, Divider } from '@mui/material';
import { SmartToy, GpsFixed, BatteryChargingFull, PlayCircle } from '@mui/icons-material';

const RobotStatus = () => {
    // This data would typically come from state management or props
    const status = {
        id: 'ROBOT-007',
        location: '[7, 7]',
        state: 'Moving to Target',
        battery: '88%',
    };

    return (
        <Paper sx={{ p: 2 }} elevation={3}>
            <Typography variant="h6" gutterBottom>Robot Status</Typography>
            <List dense>
                <ListItem>
                    <ListItemIcon>
                        <SmartToy color="primary" />
                    </ListItemIcon>
                    <ListItemText primary="Robot ID" secondary={status.id} />
                </ListItem>
                <Divider component="li" />
                <ListItem>
                    <ListItemIcon>
                        <GpsFixed color="primary" />
                    </ListItemIcon>
                    <ListItemText primary="Location (Grid)" secondary={status.location} />
                </ListItem>
                <Divider component="li" />
                <ListItem>
                    <ListItemIcon>
                        <PlayCircle color="primary" />
                    </ListItemIcon>
                    <ListItemText primary="Current State" secondary={status.state} />
                </ListItem>
                <Divider component="li" />
                <ListItem>
                    <ListItemIcon>
                        <BatteryChargingFull color="primary" />
                    </ListItemIcon>
                    <ListItemText primary="Battery Level" secondary={status.battery} />
                </ListItem>
            </List>
        </Paper>
    );
};

export default RobotStatus;
