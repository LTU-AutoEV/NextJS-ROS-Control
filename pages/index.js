import Link from 'next/link'
import TopicDisplay from 'components/topic-display.js'

import {Container, Row, Col} from 'reactstrap'

export default () => (
  <Container>
    <Row>
      <Col>
        <TopicDisplay name="Topic A" topic="/a" type="std_msgs/Float64" />
      </Col>
      <Col>
        <TopicDisplay name="Topic B" topic="/b" type="std_msgs/Float64" />
      </Col>
    </Row>
  </Container>
)
